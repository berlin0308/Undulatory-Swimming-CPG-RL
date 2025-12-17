#!/usr/bin/env python3
"""
Evaluation script for trained CPG-RL swimming robots.
Loads a trained model and evaluates performance with optional fluid disturbance.
"""

import os
import argparse
import numpy as np
import torch
from cpg_env_adapter import SwimmingCPGEnv
from cpg_ppo_agent import RunnerPPO, ACNetwork
from cpg_config_utils import RUN_PROFILES


def evaluate(
    checkpoint_path: str,
    num_episodes: int = 10,
    render: bool = True,
    fluid_disturbance: str = None,
    disturbance_intensity: float = 1.0,
    max_steps: int = 500,
):
    """
    Evaluate a trained model.

    Args:
        checkpoint_path: Path to model checkpoint (.pt file)
        num_episodes: Number of evaluation episodes
        render: Show visualization
        fluid_disturbance: Disturbance mode (none/constant/turbulent/vortex/oscillating/mixed)
        disturbance_intensity: Disturbance strength
        max_steps: Maximum steps per episode
    """
    # Load checkpoint
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path)
    print(f"Loading checkpoint: {checkpoint_path}")
    print(f"  Update: {checkpoint.get('update', 'unknown')}")

    # Extract config from checkpoint
    config = checkpoint.get('config', {})
    hidden_dim = config.get('hidden', 128)

    # Create environment
    env = SwimmingCPGEnv(
        control_mode="cpg_rl",
        show_viewer=render,
        fluid_disturbance=fluid_disturbance,
        disturbance_intensity=disturbance_intensity,
    )

    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]

    # Create network and load weights
    network = ACNetwork(
        obs_dim=obs_dim,
        act_dim=act_dim,
        hidden_dim=hidden_dim,
    )
    network.actor.load_state_dict(checkpoint['actor_state_dict'])
    network.critic.load_state_dict(checkpoint['critic_state_dict'])
    network.actor.eval()
    network.critic.eval()

    print(f"\nStarting evaluation:")
    print(f"  Episodes: {num_episodes}")
    print(f"  Render: {render}")
    print(f"  Fluid disturbance: {fluid_disturbance or 'none'}")
    if fluid_disturbance:
        print(f"  Disturbance intensity: {disturbance_intensity}")
    print()

    # Evaluation loop
    all_returns = []
    all_distances = []
    all_speeds = []

    for episode in range(num_episodes):
        obs = env.reset()
        episode_return = 0.0
        episode_steps = 0

        for step in range(max_steps):
            # Get action (deterministic)
            with torch.no_grad():
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
                action_mean, _ = network.actor(obs_tensor)
                action = action_mean.squeeze(0).cpu().numpy()

            # Step environment
            obs, reward, done, info = env.step(action)
            episode_return += reward
            episode_steps += 1

            if done:
                break

        # Get final metrics
        final_pos = info.get('position', np.zeros(3))
        distance = np.linalg.norm(final_pos[:2])  # XY distance
        avg_speed = distance / (episode_steps * env.dt) if episode_steps > 0 else 0.0

        all_returns.append(episode_return)
        all_distances.append(distance)
        all_speeds.append(avg_speed)

        print(f"Episode {episode + 1}/{num_episodes}: "
              f"return={episode_return:.3f}, "
              f"distance={distance:.3f}m, "
              f"avg_speed={avg_speed:.3f}m/s, "
              f"steps={episode_steps}")

    # Print summary statistics
    print(f"\n{'='*70}")
    print(f"Evaluation Summary ({num_episodes} episodes)")
    print(f"{'='*70}")
    print(f"Return:        {np.mean(all_returns):.3f} ± {np.std(all_returns):.3f}")
    print(f"Distance:      {np.mean(all_distances):.3f} ± {np.std(all_distances):.3f} m")
    print(f"Avg Speed:     {np.mean(all_speeds):.3f} ± {np.std(all_speeds):.3f} m/s")
    print(f"{'='*70}\n")

    env.close()


def main():
    parser = argparse.ArgumentParser(description="Evaluate trained CPG-RL swimming robot")

    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to model checkpoint (.pt file)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="Number of evaluation episodes (default: 10)"
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Show visualization during evaluation"
    )
    parser.add_argument(
        "--no-render",
        dest="render",
        action="store_false",
        help="Disable visualization (faster evaluation)"
    )
    parser.set_defaults(render=True)

    parser.add_argument(
        "--disturbance",
        type=str,
        default=None,
        choices=["none", "constant", "turbulent", "vortex", "oscillating", "mixed"],
        help="Fluid disturbance mode (default: none)"
    )
    parser.add_argument(
        "--intensity",
        type=float,
        default=1.0,
        help="Disturbance intensity multiplier (default: 1.0)"
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=500,
        help="Maximum steps per episode (default: 500)"
    )

    args = parser.parse_args()

    # Normalize disturbance argument
    disturbance = args.disturbance
    if disturbance == "none":
        disturbance = None

    evaluate(
        checkpoint_path=args.checkpoint,
        num_episodes=args.episodes,
        render=args.render,
        fluid_disturbance=disturbance,
        disturbance_intensity=args.intensity,
        max_steps=args.max_steps,
    )


if __name__ == "__main__":
    main()
