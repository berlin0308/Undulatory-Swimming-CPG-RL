#!/usr/bin/env python3
# cpg_ppo_agent.py
"""
Corrected PPO agent for CPG learning.
Fixes:
- Too-small policy std (no exploration)
- Too-small output gain (policy outputs stuck near bias)
- Low entropy coefficient
- Safe default initialization for actions
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal


# ============================================================
#   Actor–Critic Network
# ============================================================

class ACNetwork(nn.Module):
    def __init__(self, obs_dim, act_dim,
                 hidden_dim=256,
                 init_std=0.5,              # MUCH larger exploration
                 init_gain=0.05,            # higher output gain
                 action_bias_init=None):

        super().__init__()

        # Shared body
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        # ---------------------------
        # Policy head
        # ---------------------------
        self.mu = nn.Linear(hidden_dim, act_dim)
        # Some backends (e.g., MPS) do not implement torch.linalg.qr, which is used by orthogonal_.
        # Initialize on CPU and copy back to avoid device-specific limitations.
        with torch.no_grad():
            w_cpu = self.mu.weight.detach().cpu()
            nn.init.orthogonal_(w_cpu, gain=init_gain)
            self.mu.weight.copy_(w_cpu.to(self.mu.weight.device))

        if action_bias_init is not None:
            with torch.no_grad():
                self.mu.bias.copy_(torch.tensor(action_bias_init, dtype=torch.float32))
        else:
            nn.init.zeros_(self.mu.bias)

        # Learnable std — initialized high for exploration
        self.logstd = nn.Parameter(torch.ones(act_dim) * np.log(init_std))

        # ---------------------------
        # Value head
        # ---------------------------
        self.v = nn.Linear(hidden_dim, 1)
        with torch.no_grad():
            w_cpu = self.v.weight.detach().cpu()
            nn.init.orthogonal_(w_cpu, gain=1.0)
            self.v.weight.copy_(w_cpu.to(self.v.weight.device))
        nn.init.zeros_(self.v.bias)

    def forward(self, x):
        h = self.net(x)
        mu = self.mu(h)
        std = torch.exp(self.logstd)
        val = self.v(h).squeeze(-1)
        return mu, std, val

    def act_value(self, x, deterministic=False):
        mu, std, val = self.forward(x)
        dist = Normal(mu, std)

        if deterministic:
            action = mu
            logp = dist.log_prob(action).sum(-1)
        else:
            action = dist.rsample()
            logp = dist.log_prob(action).sum(-1)
        

        return action, logp, val
   



# ============================================================
#   PPO Runner
# ============================================================

class RunnerPPO:
    def __init__(self, obs_dim, act_dim, cfg):

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Action bias init for CPG RL mode
        action_bias_init = None
        if act_dim >= 5:
            action_bias_init = [
                cfg.get("freq_mod_init", 0.0),
                cfg.get("amp_mod_init", 0.0),
                cfg.get("phase_shift_init", 0.0),
                cfg.get("couple_w_forward", 1.5),
                cfg.get("couple_w_backward", 1.5),
            ]
            if act_dim == 6:
                action_bias_init.append(0.0)

        # ---------------------------
        # Create model
        # ---------------------------
        # print(act_dim) 
        self.model = ACNetwork(
            obs_dim,
            act_dim,
            hidden_dim=cfg.get("hidden", 256),
            init_std=cfg.get("init_std", 0.3),
            init_gain=cfg.get("init_gain", 0.2),
            action_bias_init=action_bias_init
        ).to(self.device)

        # ---------------------------
        # PPO configs
        # ---------------------------
        self.optim = optim.Adam(self.model.parameters(), lr=cfg.get("lr", 3e-4))
        self.clip = cfg.get("clip", 0.2)
        self.ppo_epochs = cfg.get("ppo_epochs", 5)
        self.vf_coeff = cfg.get("vf_coeff", 0.5)
        self.ent_coeff = cfg.get("ent_coeff", 0.02)  # higher entropy → more exploration
        self.batch_size = cfg.get("batch_size", 64)

    # ---------------------------------------------------------
    #   Action
    # ---------------------------------------------------------
    def act(self, obs_np, deterministic=False):
        obs = torch.tensor(obs_np, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            a, logp, v = self.model.act_value(obs, deterministic)
        return (
            a.cpu().numpy().squeeze(0),
            float(logp.cpu().numpy()),
            float(v.cpu().numpy())
        )

    # ---------------------------------------------------------
    #   PPO Update
    # ---------------------------------------------------------
    def update(self, obs, acts, old_logps, returns, advs):

        obs_t = torch.tensor(obs, dtype=torch.float32, device=self.device)
        acts_t = torch.tensor(acts, dtype=torch.float32, device=self.device)
        oldlogp_t = torch.tensor(old_logps, dtype=torch.float32, device=self.device)
        returns_t = torch.tensor(returns, dtype=torch.float32, device=self.device)
        advs_t = torch.tensor(advs, dtype=torch.float32, device=self.device)

        # Advantage normalization
        advs_t = (advs_t - advs_t.mean()) / (advs_t.std() + 1e-8)

        dataset = torch.utils.data.TensorDataset(
            obs_t, acts_t, oldlogp_t, returns_t, advs_t
        )
        loader = torch.utils.data.DataLoader(dataset, batch_size=self.batch_size, shuffle=True)

        stats = {"pi_loss": [], "vf_loss": [], "ent": []}

        for _ in range(self.ppo_epochs):
            for b_obs, b_acts, b_oldlog, b_rets, b_advs in loader:

                mu, std, val = self.model(b_obs)
                dist = Normal(mu, std)
                new_logp = dist.log_prob(b_acts).sum(-1)
                entropy = dist.entropy().sum(-1).mean()

                ratio = (new_logp - b_oldlog).exp()

                s1 = ratio * b_advs
                s2 = torch.clamp(ratio, 1.0 - self.clip, 1.0 + self.clip) * b_advs

                pi_loss = -torch.min(s1, s2).mean()
                vf_loss = ((b_rets - val) ** 2).mean()

                loss = pi_loss + self.vf_coeff * vf_loss - self.ent_coeff * entropy

                self.optim.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), 0.5)
                self.optim.step()

                stats["pi_loss"].append(pi_loss.item())
                stats["vf_loss"].append(vf_loss.item())
                stats["ent"].append(entropy.item())

        return {k: float(np.mean(v)) for k, v in stats.items()}
