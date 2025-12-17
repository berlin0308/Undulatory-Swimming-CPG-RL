#!/usr/bin/env python3
# train_run_cpg.py
"""
Main training entry. Uses the above modules to train PPO that modulates the CPG.
"""
import argparse, os, numpy as np, time
from cpg_config_utils import RUN_PROFILES, RunSaver
from cpg_env_adapter import SwimmingCPGEnv
from cpg_ppo_agent import RunnerPPO
from tqdm import tqdm
import wandb

def compute_gae(rewards, values, dones, gamma, lam):
    adv = np.zeros_like(rewards, dtype=np.float32)
    print("##################################")
    
    last = 0.0
    for t in reversed(range(len(rewards))):
        nonterm = 1.0 - float(dones[t])
        nextv = values[t+1] if t+1 < len(values) else 0.0
        delta = rewards[t] + gamma * nextv * nonterm - values[t]
        last = delta + gamma * lam * nonterm * last
        adv[t] = last
    returns = adv + values
    print(adv)
    return adv, returns

def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="short", choices=list(RUN_PROFILES.keys()))
    parser.add_argument("--mode", choices=["cpg", "direct", "cpg_rl", "res_rl"], default="direct",
                        help="Control mode: 'direct'=RL joints, 'cpg'=pure CPG, 'cpg_rl'=RL modulates CPG")
    parser.add_argument("--cpg", type=int, default=0, help="0 for PhaseOscillator, 1 for Matsuoka")
    parser.add_argument("--learn-phase-offset", action="store_true", help="Make total_phase_offset learnable")
    parser.add_argument("--obs-joint-vel", action="store_true", help="Add joint velocities (6) to observations")
    parser.add_argument("--obs-linear-vel", action="store_true", help="Add linear velocity (3) to observations")
    parser.add_argument("--obs-phase-diff", action="store_true", help="Add phase differences (5) to observations")
    parser.add_argument("--disturbance", type=str, default=None,
                        choices=[None, "constant", "turbulent", "vortex", "oscillating", "mixed"],
                        help="Fluid disturbance mode (overrides config)")
    parser.add_argument("--disturbance-intensity", type=float, default=None,
                        help="Disturbance intensity multiplier (overrides config)")
    
    # NEW: Paddle disturbance arguments
    parser.add_argument("--paddle", action="store_true",
                        help="Enable paddle disturbance")
    parser.add_argument("--paddle-count", type=int, default=2,
                        help="Number of paddles (default: 6)")
    parser.add_argument("--paddle-freq", type=float, default=0.2,
                        help="Paddle oscillation frequency in Hz (default: 1.5)")
    parser.add_argument("--paddle-amp", type=float, default=0.17,
                        help="Paddle oscillation amplitude in meters (default: 0.10)")
    parser.add_argument("--paddle-size", type=float, nargs=3, default=[0.15, 0.15, 0.05],
                        help="Paddle size [length, width, thickness] in meters (default: 0.15 0.15 0.05)")
    parser.add_argument("--paddle-mass", type=float, default=1500.0,
                        help="Paddle density in kg/m³ (default: 2000.0)")
    parser.add_argument("--swim-zone", type=float, nargs=2, default=[0.0, 1.8],
                        help="X-axis swimming zone to exclude [min_x, max_x] (default: 0.6 0.9)")
    
    parser.add_argument("--note", type=str, default="def")
    parser.add_argument("--out", default="cpg_runs")
    args = parser.parse_args()

    cfg = RUN_PROFILES[args.profile].copy()

    # Override disturbance config with command-line arguments if provided
    if args.disturbance is not None:
        cfg["fluid_disturbance"] = args.disturbance
    if args.disturbance_intensity is not None:
        cfg["disturbance_intensity"] = args.disturbance_intensity

    saver = RunSaver(root=args.out)
    saver.save_cfg(cfg)

    run_name = saver.run_name

    # Add configuration suffix to run name
    suffix_parts = []
    suffix_parts.append(args.mode)  # cpg_rl, direct, or cpg
    if args.mode == "cpg_rl":
        if args.learn_phase_offset:
            suffix_parts.append("learnable_offset")
    if args.obs_joint_vel:
        suffix_parts.append("jvel")
    if args.obs_linear_vel:
        suffix_parts.append("lvel")
    if args.obs_phase_diff:
        suffix_parts.append("pdiff")
    if args.paddle:
        suffix_parts.append(f"paddle{args.paddle_count}")  # NEW: Add paddle info to run name
    if args.note:
        suffix_parts.append(args.note)

    if suffix_parts:
        run_name = f"{run_name}_{'_'.join(suffix_parts)}"

    # NEW: Configure paddle disturbance
    paddle_config = None
    if args.paddle:
        paddle_config = {
            'n_paddles': args.paddle_count,
            'oscillation_freq': args.paddle_freq,
            'oscillation_amplitude': args.paddle_amp,
            'paddle_size': tuple(args.paddle_size),
            'paddle_mass': args.paddle_mass,
            'swim_zone': tuple(args.swim_zone),
        }
        print(f"\n{'='*70}")
        print(f"🌊 Paddle Disturbance Configuration")
        print(f"{'='*70}")
        print(f"  Enabled: True")
        print(f"  Number of paddles: {paddle_config['n_paddles']}")
        print(f"  Oscillation frequency: {paddle_config['oscillation_freq']} Hz")
        print(f"  Oscillation amplitude: {paddle_config['oscillation_amplitude']} m")
        print(f"  Paddle size: {paddle_config['paddle_size']} m")
        print(f"  Paddle mass: {paddle_config['paddle_mass']} kg/m³")
        print(f"  Swim zone (X-axis excluded): x ∈ [{paddle_config['swim_zone'][0]:.2f}, {paddle_config['swim_zone'][1]:.2f}]")
        print(f"{'='*70}\n")

    run = wandb.init(
        # Set the wandb entity where your project will be logged (generally your team name).
        entity="junzhehu-carnegie-mellon-university",
        # Set the wandb project where this run will be logged.
        project="hydro-self",
        # Track hyperparameters and run metadata.
        config={**cfg, "paddle_enabled": args.paddle, "paddle_config": paddle_config if paddle_config else None},
        name=run_name
    )

    env = SwimmingCPGEnv(
        control_mode=args.mode,
        cpg_model_type=args.cpg,
        couple=True,
        couple_w_forward=cfg.get("couple_w_forward", 1.0),
        couple_w_backward=cfg.get("couple_w_backward", 1.0),
        learn_phase_offset=args.learn_phase_offset,
        obs_joint_vel=args.obs_joint_vel,
        obs_linear_vel=args.obs_linear_vel,
        obs_phase_diff=args.obs_phase_diff,
        debug=True,
        record=False,
        fluid_disturbance=cfg.get("fluid_disturbance", None),
        disturbance_intensity=cfg.get("disturbance_intensity", 1.0),
        cpg_freq=cfg.get("cpg_freq", 2.0),
        cpg_alpha=cfg.get("cpg_alpha", 1.5),
        cpg_dt=cfg.get("cpg_dt", None),
        res_rl_residual_scale=cfg.get("res_rl_residual_scale", 0.1),
        res_rl_residual_penalty=cfg.get("res_rl_residual_penalty", 0.0),
        terminate_on_x_boundary=cfg.get("terminate_on_x_boundary", True),
        x_left_boundary=cfg.get("x_left_boundary", 0.3),
        x_right_boundary=cfg.get("x_right_boundary", 1.6),
        x_boundary_penalty=cfg.get("x_boundary_penalty", 50.0),
        # NEW: Paddle disturbance parameters
        enable_paddle_disturbance=args.paddle,
        paddle_disturbance_config=paddle_config,
    )

    obs_dim = env.obs_dim
    act_dim = env.action_dim

    print(f"🔍 Debug: obs_dim={obs_dim}, act_dim={act_dim}")
    print(f"🔍 Debug: obs_joint_vel={args.obs_joint_vel}, obs_linear_vel={args.obs_linear_vel}, obs_phase_diff={args.obs_phase_diff}")

    # Override init_std and init_gain for res_rl mode (ControlNet-style zero initialization)
    if args.mode == "res_rl":
        cfg["init_std"] = cfg.get("res_rl_init_std", 0.05)
        cfg["init_gain"] = cfg.get("res_rl_init_gain", 0.01)
        print(f"🔧 res_rl mode: Using zero-like initialization (init_std={cfg['init_std']}, init_gain={cfg['init_gain']})")

    ppo = RunnerPPO(obs_dim, act_dim, cfg)
    updates = cfg["updates"]
    batch_steps = cfg["batch_steps"]
    gamma = cfg["gamma"]
    lam = cfg["gae_lambda"]

    obs = env.reset()
    print(f"🔍 Debug: First observation shape: {obs.shape}, expected obs_dim: {obs_dim}")
    total_steps = 0

    # Track learnable weights over time (only for cpg_rl mode)
    if args.mode == "cpg_rl":
        weight_history = {
            "freq_mod": [],
            "amp_mod": [],
            "phase_shift": [],
            "w_forward": [],
            "w_backward": [],
        }
        if args.learn_phase_offset:
            weight_history["total_phase_offset"] = []
    else:
        weight_history = None

    # Episode tracking
    episode_lengths = []
    current_episode_length = 0

    for update in tqdm(range(1, updates+1)):
        # collect batch
        obs_buf, act_buf, logp_buf, rew_buf, val_buf, done_buf = [], [], [], [], [], []
        # Accumulate info for logging
        info_buf = []
        print("Entering training loop with updates =", updates, "and batch_steps =", batch_steps, flush=True)

        for step in tqdm(range(batch_steps)):
            action, logp, val = ppo.act(obs)
            print("before step############")
            next_obs, reward, done, info = env.step(action, sim_steps=1)
            print("after step############")
            obs_buf.append(obs.copy()); act_buf.append(action.copy()); logp_buf.append(logp)
            rew_buf.append(reward); val_buf.append(val); done_buf.append(done)
            info_buf.append(info)
            print(f"Reward at step {step}: {reward}")
            obs = next_obs
            total_steps += 1
            current_episode_length += 1
            print(done)
            if done:
                episode_lengths.append(current_episode_length)
                current_episode_length = 0
                obs = env.reset()
        print("⚠️ System NOT fully controllable/observable.")
        obs_arr = np.array(obs_buf)
        acts_arr = np.array(act_buf)
        old_logp = np.array(logp_buf, dtype=np.float32)
        rewards = np.array(rew_buf, dtype=np.float32)
        values = np.array(val_buf, dtype=np.float32)
        dones = np.array(done_buf, dtype=bool)

        # compute advantages
        
        advs, returns = compute_gae(rewards, values, dones, gamma, lam)

        stats = ppo.update(obs_arr, acts_arr, old_logp, returns, advs)

        avg_reward = float(np.sum(rewards) / max(1, np.count_nonzero(~dones)))

        # Compute mean reward terms from info_buf
        r_forward_vals = [info.get("reward_terms", {}).get("r_forward", 0.0) for info in info_buf]
        r_phase_vals = [info.get("reward_terms", {}).get("r_phase", 0.0) for info in info_buf]
        r_smooth_vals = [info.get("reward_terms", {}).get("r_smooth", 0.0) for info in info_buf]
        r_gait_vals = [info.get("reward_terms", {}).get("r_gait", 0.0) for info in info_buf]
        r_amp_vals = [info.get("reward_terms", {}).get("r_amp", 0.0) for info in info_buf]
        r_head_vals = [info.get("reward_terms", {}).get("r_head", 0.0) for info in info_buf]
        r_wave_vals = [info.get("reward_terms", {}).get("r_wave", 0.0) for info in info_buf]
        lateral_penalty_vals = [info.get("reward_terms", {}).get("lateral_penalty", 0.0) for info in info_buf]
        residual_penalty_vals = [info.get("reward_terms", {}).get("residual_penalty", 0.0) for info in info_buf]
        energy_reward_vals = [info.get("reward_terms", {}).get("energy_reward", 0.0) for info in info_buf]
        r_forward_disp_vals = [info.get("reward_terms", {}).get("r_forward_disp", 0.0) for info in info_buf]

        # Compute mean reward components from info_buf
        alive_reward_vals = [info.get("reward_components", {}).get("alive", 0.0) for info in info_buf]
        energy_penalty_vals = [info.get("reward_components", {}).get("energy_pen", 0.0) for info in info_buf]

        # Compute mean head position from info_buf
        head_x_vals = [info.get("head_position", {}).get("x", 0.0) for info in info_buf]
        head_y_vals = [info.get("head_position", {}).get("y", 0.0) for info in info_buf]
        head_z_vals = [info.get("head_position", {}).get("z", 0.0) for info in info_buf]

        # Compute mean lateral displacement from info_buf
        lateral_disp_vals = [info.get("lateral_disp", 0.0) for info in info_buf]

        # Compute mean residual statistics for res_rl mode
        residual_mean_vals = [info.get("residual_stats", {}).get("residual_mean", 0.0) for info in info_buf]
        residual_max_vals = [info.get("residual_stats", {}).get("residual_max", 0.0) for info in info_buf]
        residual_std_vals = [info.get("residual_stats", {}).get("residual_std", 0.0) for info in info_buf]

        # Compute mean phase statistics for CPG modes
        phase_mean_vals = [info.get("phase_data", {}).get("phase_mean", 0.0) for info in info_buf if "phase_data" in info]
        phase_std_vals = [info.get("phase_data", {}).get("phase_std", 0.0) for info in info_buf if "phase_data" in info]

        # Track and log learnable weights (cpg_rl mode)
        log_data = {
            "avg_reward": avg_reward,
            "pi_loss": stats["pi_loss"],
            "vf_loss": stats["vf_loss"],
            "r_forward": float(np.mean(r_forward_vals)),
            "r_phase": float(np.mean(r_phase_vals)),
            "r_smooth": float(np.mean(r_smooth_vals)),
            "r_gait": float(np.mean(r_gait_vals)),
            "r_amp": float(np.mean(r_amp_vals)),
            "r_head": float(np.mean(r_head_vals)),
            "r_wave": float(np.mean(r_wave_vals)),
            "lateral_penalty": float(np.mean(lateral_penalty_vals)),
            "residual_penalty": float(np.mean(residual_penalty_vals)),
            "alive_reward": float(np.mean(alive_reward_vals)),
            "energy_penalty": float(np.mean(energy_penalty_vals)),
            "energy_reward": float(np.mean(energy_reward_vals)),  # NEW: energy reward from joint motion
            "r_forward_disp": float(np.mean(r_forward_disp_vals)),  # NEW: forward displacement reward
            "head_x": float(np.mean(head_x_vals)),
            "head_y": float(np.mean(head_y_vals)),
            "head_z": float(np.mean(head_z_vals)),
            "lateral_disp": float(np.mean(lateral_disp_vals)),  # NEW: lateral displacement from center
        }

        # Add episode length statistics
        if len(episode_lengths) > 0:
            log_data["episode_length_mean"] = float(np.mean(episode_lengths))
            log_data["episode_length_min"] = float(np.min(episode_lengths))
            log_data["episode_length_max"] = float(np.max(episode_lengths))
            log_data["episode_length_std"] = float(np.std(episode_lengths))
            log_data["num_episodes"] = len(episode_lengths)
            # Clear episode lengths after logging
            episode_lengths = []

        # Add residual statistics for res_rl mode
        if args.mode == "res_rl":
            log_data["residual_mean"] = float(np.mean(residual_mean_vals))
            log_data["residual_max"] = float(np.mean(residual_max_vals))
            log_data["residual_std"] = float(np.mean(residual_std_vals))

        # Add phase statistics for CPG modes (cpg, cpg_rl, res_rl)
        if len(phase_mean_vals) > 0:
            log_data["phase_mean"] = float(np.mean(phase_mean_vals))
            log_data["phase_std"] = float(np.mean(phase_std_vals))

        if weight_history is not None:
            # Compute mean of CPG parameters from this batch
            mean_freq = float(np.mean(acts_arr[:, 0]))
            mean_amp = float(np.mean(acts_arr[:, 1]))
            mean_phase = float(np.mean(acts_arr[:, 2]))
            mean_wf = float(np.mean(acts_arr[:, 3]))
            mean_wb = float(np.mean(acts_arr[:, 4]))

            weight_history["freq_mod"].append(mean_freq)
            weight_history["amp_mod"].append(mean_amp)
            weight_history["phase_shift"].append(mean_phase)
            weight_history["w_forward"].append(mean_wf)
            weight_history["w_backward"].append(mean_wb)

            log_data["freq_mod"] = mean_freq
            log_data["amp_mod"] = mean_amp
            log_data["phase_shift"] = mean_phase
            log_data["w_forward"] = mean_wf
            log_data["w_backward"] = mean_wb

            if args.learn_phase_offset:
                mean_offset = float(np.mean(acts_arr[:, 5]))
                weight_history["total_phase_offset"].append(mean_offset)
                log_data["total_phase_offset"] = mean_offset

        saver.log(update, log_data)
        run.log({"Update_step": update, **log_data})

        if update % cfg["log_every"] == 0:
            # Print episode length stats if available
            ep_len_str = ""
            if "episode_length_mean" in log_data:
                ep_len_str = f" ep_len={log_data['episode_length_mean']:.1f}±{log_data['episode_length_std']:.1f} (n={log_data['num_episodes']})"

            if weight_history is not None:
                print(f"[{update}/{updates}] avg_reward={avg_reward:.3f} pi_loss={stats['pi_loss']:.4f}{ep_len_str}")
                print(f"  freq={log_data['freq_mod']:.3f} amp={log_data['amp_mod']:.3f} phase={log_data['phase_shift']:.3f}")
                if args.learn_phase_offset:
                    print(f"  w_f={log_data['w_forward']:.3f} w_b={log_data['w_backward']:.3f} offset={log_data['total_phase_offset']:.3f}")
                else:
                    print(f"  w_f={log_data['w_forward']:.3f} w_b={log_data['w_backward']:.3f}")
            elif args.mode == "res_rl":
                print(f"[{update}/{updates}] avg_reward={avg_reward:.3f} pi_loss={stats['pi_loss']:.4f}{ep_len_str}")
                print(f"  residual: mean={log_data['residual_mean']:.4f} max={log_data['residual_max']:.4f} std={log_data['residual_std']:.4f}")
            else:
                print(f"[{update}/{updates}] avg_reward={avg_reward:.3f} pi_loss={stats['pi_loss']:.4f}{ep_len_str}")

        if update % cfg["save_every"] == 0:
            torch_path = os.path.join(saver.dir, f"model_up{update}.pt")
            try:
                import torch
                torch.save({"model": ppo.model.state_dict(), "cfg": cfg}, torch_path)
                print("Saved checkpoint:", torch_path)
            except Exception as e:
                print("Failed to save:", e)

    # final save + plot
    try:
        import torch
        torch.save({"model": ppo.model.state_dict(), "cfg": cfg}, os.path.join(saver.dir, "model_final.pt"))
    except:
        pass

    saver.plot_rewards()

    # Plot coupling weights if in cpg_rl mode
    if weight_history is not None:
        saver.plot_coupling_weights()
        print(f"\n📊 Final learned CPG parameters:")
        print(f"  freq_mod (mean): {np.mean(weight_history['freq_mod']):.3f}")
        print(f"  amp_mod (mean): {np.mean(weight_history['amp_mod']):.3f}")
        print(f"  phase_shift (mean): {np.mean(weight_history['phase_shift']):.3f}")
        print(f"  w_forward (mean): {np.mean(weight_history['w_forward']):.3f}")
        print(f"  w_backward (mean): {np.mean(weight_history['w_backward']):.3f}")
        if "total_phase_offset" in weight_history:
            print(f"  total_phase_offset (mean): {np.mean(weight_history['total_phase_offset']):.3f}")
        print(f"  Coupling asymmetry: {np.mean(weight_history['w_forward']) - np.mean(weight_history['w_backward']):.3f}")

    run.finish()
    print("Training done. Outputs in:", saver.dir)

if __name__ == "__main__":
    main()