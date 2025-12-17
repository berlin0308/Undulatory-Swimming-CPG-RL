#!/usr/bin/env python3
# cpg_config_utils.py
"""
Configurations and simple logging utilities for CPG+PPO.
"""

import os, json
from datetime import datetime
import matplotlib.pyplot as plt

RUN_PROFILES = {
    "short": {
        "lr": 1e-4,
        "batch_steps": 200,
        "hidden": 128,
        "max_episode_steps": 500,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip": 0.2,
        "ppo_epochs": 4,
        "vf_coeff": 0.5,
        "ent_coeff": 0.05,
        "updates": 500,
        "save_every": 20,
        "log_every": 5,
        # CPG parameter initial values
        "freq_mod_init": 0.0,        # Frequency modulation starting value
        "amp_mod_init": 0.0,         # Amplitude modulation starting value
        "phase_shift_init": 0.0,     # Phase shift starting value
        "couple_w_forward": 0.5,
        "couple_w_backward": 0.5,
        "total_phase_offset_init": 2 * 3.14159265359,  # 2π
        # CPG network default parameters
        "cpg_freq": 2.0,             # CPG base frequency in Hz
        "cpg_alpha": 1.5,            # CPG coupling strength
        "cpg_dt": 0.05,              # CPG time step (should match env dt)
        # res_rl specific parameters (ControlNet-style zero initialization)
        "res_rl_init_std": 0.5,     # Small std for near-zero residuals initially
        "res_rl_init_gain": 0.2,    # Small gain for near-zero residuals initially
        "res_rl_residual_scale": 1,  # Maximum residual scaling (used with tanh)
        "res_rl_residual_penalty": 0.01,  # L2 penalty on residual magnitude
        # Fluid disturbance parameters (for robustness training/evaluation)
        "fluid_disturbance": "turbulent",         # Disturbance mode: None, "constant", "turbulent", "vortex", "oscillating", "mixed"
        "disturbance_intensity": 25.0, # 50.0,      # Disturbance intensity multiplier (50.0 = strong, 25.0 = standard)
        "disturbance_randomize": True,     # Randomize disturbance parameters for domain randomization
        # Boundary termination parameters
        "terminate_on_x_boundary": True, #nate episode when x goes outside boundaries
        "x_left_boundary": 0.7,    # Left boundary (x < x_left triggers termination)
        "x_right_boundary": 1.7,#1.3     # Right boundary (x > x_right triggers termination)
        "x_boundary_penalty": 50.0,        # Penalty applied when hitting x boundary
    },
    "standard": {
        "lr": 3e-3,
        "batch_steps": 204,
        "hidden": 256,
        "max_episode_steps": 1000,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip": 0.2,
        "ppo_epochs": 4,
        "vf_coeff": 0.5,
        "ent_coeff": 0.01,
        "updates": 1000,
        "save_every": 200,
        "log_every": 10,
        # CPG parameter initial values
        "freq_mod_init": 0.0,        # Frequency modulation starting value
        "amp_mod_init": 0.0,         # Amplitude modulation starting value
        "phase_shift_init": 0.0,     # Phase shift starting value
        "couple_w_forward": 1.0,
        "couple_w_backward": 1.0,
        "total_phase_offset_init": 2 * 3.14159265359,  # 2π
        # CPG network default parameters
        "cpg_freq": 2.0,             # CPG base frequency in Hz
        "cpg_alpha": 1.5,            # CPG coupling strength
        "cpg_dt": 0.05,              # CPG time step (should match env dt)
        # res_rl specific parameters (ControlNet-style zero initialization)
        "res_rl_init_std": 0.05,     # Small std for near-zero residuals initially
        "res_rl_init_gain": 0.01,    # Small gain for near-zero residuals initially
        "res_rl_residual_scale": 0.7,  # Maximum residual scaling (used with tanh)
        "res_rl_residual_penalty": 0.001,  # L2 penalty on residual magnitude
        # Fluid disturbance parameters (for robustness training/evaluation)
        "fluid_disturbance": None,         # Disturbance mode: None, "constant", "turbulent", "vortex", "oscillating", "mixed"
        "disturbance_intensity": 1.0,      # Disturbance intensity multiplier (0.0 = off, 1.0 = standard)
        "disturbance_randomize": True,     # Randomize disturbance parameters for domain randomization
        # Boundary termination parameters
        "terminate_on_x_boundary": True,   # Terminate episode when x goes outside boundaries
        "x_left_boundary": 0.3,           # Left boundary (x < x_left triggers termination)
        "x_right_boundary": 1.8,           # Right boundary (x > x_right triggers termination)
        "x_boundary_penalty": 50.0,        # Penalty applied when hitting x boundary
    }
}

class RunSaver:
    def __init__(self, root="cpg_output", run_name=None):
        os.makedirs(root, exist_ok=True)
        if run_name is None:
            run_name = "run_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(root, run_name)
        os.makedirs(self.dir, exist_ok=True)
        self.history = []
        self.run_name = run_name

    def save_cfg(self, cfg):
        with open(self.dir + "/config.json", "w") as f:
            json.dump(cfg, f, indent=2)

    def log(self, update, metrics):
        entry = {"update": update, "time": datetime.now().isoformat(), **metrics}
        self.history.append(entry)
        with open(self.dir + "/metrics.json", "w") as f:
            json.dump(self.history, f, indent=2)

    def plot_rewards(self):
        if not self.history:
            return
        updates = [h["update"] for h in self.history]
        rewards = [h.get("avg_reward", 0.0) for h in self.history]
        plt.figure(figsize=(8,4))
        plt.plot(updates, rewards, '-o')
        plt.xlabel("update")
        plt.ylabel("avg_reward")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.dir + "/reward_curve.png")
        plt.close()

    def plot_coupling_weights(self):
        """Plot evolution of learnable coupling weights (cpg_rl mode only)."""
        if not self.history:
            return

        # Check if weight data exists
        if "w_forward" not in self.history[0]:
            return

        updates = [h["update"] for h in self.history]
        w_forward = [h.get("w_forward", 0.0) for h in self.history]
        w_backward = [h.get("w_backward", 0.0) for h in self.history]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot both weights
        ax1.plot(updates, w_forward, '-o', label='w_forward', color='blue', markersize=3)
        ax1.plot(updates, w_backward, '-s', label='w_backward', color='red', markersize=3)
        ax1.set_xlabel("Update")
        ax1.set_ylabel("Coupling Weight")
        ax1.set_title("Learned Coupling Weights Over Training")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot difference (asymmetry)
        asymmetry = [wf - wb for wf, wb in zip(w_forward, w_backward)]
        ax2.plot(updates, asymmetry, '-o', color='green', markersize=3)
        ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5)
        ax2.set_xlabel("Update")
        ax2.set_ylabel("w_forward - w_backward")
        ax2.set_title("Coupling Asymmetry (>0: Forward-biased, <0: Backward-biased)")
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.dir + "/coupling_weights.png", dpi=150)
        plt.close()

        print(f"✅ Saved coupling weights plot to {self.dir}/coupling_weights.png")
