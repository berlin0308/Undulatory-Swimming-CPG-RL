#!/usr/bin/env python3
"""
Fluid disturbance implementation for swimming robot training.
Adds random currents and turbulence directly to SPH particles.
"""

import numpy as np
from typing import Tuple, Optional


class FluidDisturbance:
    """
    Generate fluid disturbance by manipulating SPH particle velocities.

    Supports multiple disturbance modes:
    - Constant current (steady flow)
    - Turbulent fluctuations (random eddies)
    - Vortices (rotational flows)
    - Wave-like oscillations
    """

    def __init__(
        self,
        mode: str = "turbulent",
        intensity: float = 1.0,
        current_direction: Optional[np.ndarray] = None,
        frequency: float = 0.5,
        enable: bool = True,
        randomize: bool = True,
    ):
        """
        Initialize fluid disturbance generator.

        Args:
            mode: Disturbance type
                - "none": No disturbance
                - "constant": Steady current in one direction
                - "turbulent": Random fluctuations (Gaussian noise)
                - "vortex": Rotational flow pattern
                - "oscillating": Sinusoidal waves
                - "mixed": Combination of all
            intensity: Disturbance strength multiplier [0, 10] (m/s)
            current_direction: Mean flow direction [x, y, z] (normalized)
            frequency: Oscillation frequency for wave-like disturbances (Hz)
            enable: Enable/disable disturbance
            randomize: Randomize parameters each episode
        """
        self.mode = mode
        self.intensity = intensity
        self.frequency = frequency
        self.enable = enable
        self.randomize = randomize

        # Default current direction: along +Y (forward swimming direction)
        if current_direction is None:
            self.current_direction = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        else:
            self.current_direction = np.array(current_direction, dtype=np.float32)
            # Normalize
            norm = np.linalg.norm(self.current_direction)
            if norm > 1e-6:
                self.current_direction /= norm

        # Internal state
        self.time = 0.0
        self.dt = 0.05  # Will be updated from environment
        self.vortex_center = np.array([0.75, 1.0, 0.5], dtype=np.float32)  # Center of domain
        self.vortex_omega = 2.0 * np.pi  # Angular velocity

        # Randomization ranges
        self.intensity_range = (0.3, 1.5)
        self.direction_noise = 0.5  # Radians

    def reset(self, dt: float = 0.05):
        """Reset internal state (called at episode start)."""
        self.time = 0.0
        self.dt = dt

        if self.randomize:
            # Randomize intensity
            self.intensity = np.random.uniform(*self.intensity_range)

            # Randomize current direction
            if self.mode in ["constant", "mixed"]:
                # Add random rotation to base direction
                theta = np.random.uniform(-self.direction_noise, self.direction_noise)
                phi = np.random.uniform(-self.direction_noise, self.direction_noise)

                # Simple rotation (around z-axis)
                cos_t, sin_t = np.cos(theta), np.sin(theta)
                rot_z = np.array([
                    [cos_t, -sin_t, 0],
                    [sin_t, cos_t, 0],
                    [0, 0, 1]
                ])
                self.current_direction = rot_z @ self.current_direction

            # Randomize vortex center
            if self.mode in ["vortex", "mixed"]:
                self.vortex_center = np.array([
                    np.random.uniform(0.5, 1.0),
                    np.random.uniform(0.75, 1.25),
                    np.random.uniform(0.3, 0.7)
                ], dtype=np.float32)
                self.vortex_omega = np.random.uniform(1.0, 3.0) * np.pi

    def apply_disturbance(self, water_entity) -> None:
        """
        Apply disturbance to SPH particles by modifying their velocities.

        Args:
            water_entity: Genesis SPH entity (water)
        """
        if not self.enable or self.mode == "none":
            return

        # Get current particle positions and velocities
        try:
            positions = water_entity.get_particles_pos().cpu().numpy()  # (N, 3)
            velocities = water_entity.get_particles_vel().cpu().numpy()  # (N, 3)
        except (AttributeError, RuntimeError) as e:
            # SPH entity might not be fully initialized yet
            import warnings
            warnings.warn(f"Failed to access SPH particles: {e}. Skipping disturbance.")
            return

        n_particles = positions.shape[0]

        # Compute disturbance velocity field
        delta_v = np.zeros_like(velocities, dtype=np.float32)

        # --- Constant current component ---
        if self.mode in ["constant", "mixed"]:
            # Steady flow in one direction
            # Reduced from 0.5 to 0.1 for gentler current
            delta_v += self.intensity * 0.3 * self.current_direction

        # --- Turbulent fluctuations ---
        if self.mode in ["turbulent", "mixed"]:
            # Random Gaussian noise (models eddies and turbulence)
            # Reduced from 0.2 to 0.05 for smoother turbulence
            turbulent_velocity = self.intensity * 0.15 * np.random.randn(n_particles, 3).astype(np.float32)
            delta_v += turbulent_velocity

        # --- Vortex (rotational flow) ---
        if self.mode in ["vortex", "mixed"]:
            # Circular flow pattern around vortex center
            r_vec = positions - self.vortex_center  # (N, 3)
            r_norm = np.linalg.norm(r_vec, axis=1, keepdims=True)  # (N, 1)

            # Tangential velocity: v = ω × r (perpendicular in xy-plane)
            tangent = np.zeros_like(r_vec, dtype=np.float32)
            tangent[:, 0] = -r_vec[:, 1]  # vx = -dy
            tangent[:, 1] = r_vec[:, 0]   # vy = dx
            tangent[:, 2] = 0.0            # vz = 0

            # Normalize and scale by vortex strength
            tangent_norm = np.linalg.norm(tangent, axis=1, keepdims=True)
            safe_mask = (tangent_norm.squeeze() > 1e-3)

            if np.any(safe_mask):
                tangent[safe_mask] /= tangent_norm[safe_mask]
                # Reduced from 0.3 to 0.08 for gentler vortex
                vortex_velocity = self.intensity * 0.24 * tangent / (1.0 + r_norm)
                delta_v += vortex_velocity

        # --- Oscillating waves ---
        if self.mode in ["oscillating", "mixed"]:
            # Sinusoidal velocity varying with time and space
            phase = 2.0 * np.pi * self.frequency * self.time

            # Spatial wave pattern (varies along y-axis)
            spatial_wave = np.sin(2 * np.pi * positions[:, 1] / 0.5 + phase)  # (N,)

            # Reduced from 0.2 to 0.05 for gentler waves
            wave_amplitude = self.intensity * 0.15

            # Lateral oscillation (perpendicular to mean flow)
            wave_velocity = np.zeros((n_particles, 3), dtype=np.float32)
            wave_velocity[:, 0] = wave_amplitude * spatial_wave  # Lateral
            wave_velocity[:, 1] = wave_amplitude * 0.3 * np.cos(phase)  # Some forward component
            wave_velocity[:, 2] = wave_amplitude * 0.5 * np.sin(2 * phase)  # Vertical at double frequency

            delta_v += wave_velocity

        # Apply disturbance (add to existing velocity)
        new_velocities = velocities + delta_v

        # Clip velocities to prevent numerical instability
        # Maximum reasonable fluid velocity: ~10 m/s
        max_vel = 10.0
        new_velocities = np.clip(new_velocities, -max_vel, max_vel)

        # Set modified velocities back to particles
        import torch
        device = water_entity.get_particles_vel().device
        water_entity.set_particles_vel(torch.from_numpy(new_velocities).to(device))

        # Update time
        self.time += self.dt

    def get_current_info(self) -> dict:
        """
        Get current disturbance parameters (for logging/visualization).

        Returns:
            Dictionary with disturbance info
        """
        return {
            "mode": self.mode,
            "intensity": float(self.intensity),
            "direction": self.current_direction.tolist(),
            "time": float(self.time),
            "vortex_center": self.vortex_center.tolist() if self.mode in ["vortex", "mixed"] else None,
        }


# Example usage and test
if __name__ == "__main__":
    import genesis as gs

    print("="*70)
    print("Testing Fluid Disturbance with Genesis SPH")
    print("="*70)

    # Initialize Genesis
    gs.init(backend=gs.cuda, logging_level='warning')

    # Create scene with SPH
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.05, substeps=100),
        sph_options=gs.options.SPHOptions(
            lower_bound=(0.0, 0.0, 0.0),
            upper_bound=(1.5, 2.0, 1.0),
            particle_size=0.03
        ),
        show_viewer=False
    )

    # Add water
    water = scene.add_entity(
        material=gs.materials.SPH.Liquid(rho=1000.0, mu=0.0001),
        morph=gs.morphs.Box(pos=(0.75, 1.0, 0.2), size=(1.5, 2.0, 0.4)),
    )

    scene.build()

    # Test different modes
    modes = ["constant", "turbulent", "vortex", "oscillating", "mixed"]

    for mode in modes:
        print(f"\n{'='*70}")
        print(f"Mode: {mode}")
        print(f"{'='*70}")

        disturbance = FluidDisturbance(
            mode=mode,
            intensity=1.0,
            frequency=0.5,
            enable=True,
            randomize=False
        )

        disturbance.reset(dt=0.05)

        print(f"Initial particle count: {water.n_particles}")

        # Apply disturbance for a few steps
        print("\nApplying disturbance over 5 time steps:")
        for step in range(5):
            # Get initial velocity stats
            vel_before = water.get_particles_vel().cpu().numpy()
            mean_vel_before = np.mean(np.linalg.norm(vel_before, axis=1))

            # Apply disturbance
            disturbance.apply_disturbance(water)

            # Get new velocity stats
            vel_after = water.get_particles_vel().cpu().numpy()
            mean_vel_after = np.mean(np.linalg.norm(vel_after, axis=1))

            print(f"  Step {step}: mean_vel_before={mean_vel_before:.4f} m/s, "
                  f"mean_vel_after={mean_vel_after:.4f} m/s, "
                  f"Δv={mean_vel_after - mean_vel_before:.4f} m/s")

            # Step simulation
            scene.step()

        info = disturbance.get_current_info()
        print(f"\nDisturbance info: {info}")

    print(f"\n{'='*70}")
    print("Test complete!")
    print("="*70)
