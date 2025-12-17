# CPG-RL Swimming Robot

Hybrid CPG-RL control for robotic fish swimming in fluid environments with realistic disturbances.

## Features

- **Hybrid Control**: Central Pattern Generator (CPG) with Reinforcement Learning modulation
- **Kuramoto Coupling**: Phase-coupled oscillators with learnable forward/backward weights
- **Fluid Disturbance**: Realistic SPH-based water currents (constant, turbulent, vortex, oscillating, mixed)
- **Genesis Physics**: High-fidelity SPH fluid simulation with CUDA acceleration

## Installation

### Prerequisites

- Python 3.8+
- CUDA-capable GPU
- Genesis physics simulator

### Install Genesis

```bash
pip install genesis-world
```

### Install Dependencies

```bash
pip install torch numpy matplotlib
```

## Quick Start

### Training

Train with default CPG-RL mode (no disturbance):

```bash
python3 run.py --mode cpg_rl --cpg 0 --profile short
```

Train with fluid disturbance (mixed mode):

```bash
python3 run.py --mode cpg_rl --cpg 0 --profile short \
    --fluid_disturbance mixed --disturbance_intensity 1.0
```

Training profiles:
- `short`: 100 updates, quick testing (default)
- `standard`: 500 updates, full training
- `long`: 1000 updates, extended training

### Evaluation

Evaluate a trained model without disturbance:

```bash
python3 eval.py --checkpoint cpg_runs/cpg_rl_cpg0_short/model_update_100.pt \
    --episodes 10 --render
```

Evaluate with fluid disturbance:

```bash
python3 eval.py --checkpoint cpg_runs/cpg_rl_cpg0_short/model_update_100.pt \
    --episodes 10 --disturbance mixed --intensity 1.5
```

## Control Modes

### CPG-RL Mode (`--mode cpg_rl`)

RL modulates CPG parameters in real-time:

- **Action Space (5D)**: `[freq_mod, amp_mod, phase_shift, w_forward, w_backward]`
- **freq_mod**: Frequency modulation
- **amp_mod**: Amplitude modulation
- **phase_shift**: Phase offset
- **w_forward**: Forward coupling weight (head → tail)
- **w_backward**: Backward coupling weight (tail → head)

### CPG Selection

Choose CPG architecture with `--cpg` flag:

- `0`: Kuramoto (phase-coupled oscillators, **recommended**)
- `1`: Van der Pol (relaxation oscillator)
- `2`: Matsuoka (neural oscillator)

## Fluid Disturbance Modes

Enable realistic water currents during training/evaluation:

- `none`: No disturbance (still water)
- `constant`: Steady current in one direction
- `turbulent`: Random fluctuations (Gaussian noise)
- `vortex`: Rotational flow patterns
- `oscillating`: Sinusoidal waves
- `mixed`: Combination of all (most realistic, **recommended**)

Adjust intensity with `--disturbance_intensity` (default: 1.0):
- `0.5`: Gentle flow
- `1.0`: Moderate flow
- `2.0`: Strong currents

## File Structure

```
cpg_swimming_release/
├── run.py                   # Training script
├── eval.py                  # Evaluation script
├── cpg_ppo_agent.py         # PPO implementation
├── cpg_env_adapter.py       # Environment wrapper
├── fluid_disturbance.py     # SPH disturbance system
├── cpg_config_utils.py      # Configuration profiles
├── env/
│   ├── cpg.py              # CPG models (Kuramoto, Van der Pol, Matsuoka)
│   └── swim_gs_env.py      # Genesis SPH swimming environment
└── README.md               # This file
```

## Training Configuration

Modify `cpg_config_utils.py` to customize training parameters:

```python
RUN_PROFILES = {
    "my_experiment": {
        "lr": 3e-3,               # Learning rate
        "batch_steps": 500,       # Steps per update
        "hidden": 128,            # Hidden layer size
        "max_episode_steps": 500, # Episode length
        "updates": 500,           # Total updates
        "couple_w_forward": 1.0,  # Initial forward coupling
        "couple_w_backward": 1.0, # Initial backward coupling
    }
}
```

Run with custom profile:

```bash
python3 run.py --mode cpg_rl --profile my_experiment
```

## Examples

### Train robust policy with disturbance

```bash
python3 run.py --mode cpg_rl --cpg 0 --profile standard \
    --fluid_disturbance mixed --disturbance_intensity 1.0 \
    --out cpg_runs/robust_swimmer
```

### Evaluate on harder disturbance

```bash
python3 eval.py --checkpoint cpg_runs/robust_swimmer/model_update_500.pt \
    --episodes 20 --disturbance mixed --intensity 2.0 --render
```

### Train without disturbance, test with disturbance

```bash
# Train in still water
python3 run.py --mode cpg_rl --cpg 0 --profile standard

# Test in flowing water
python3 eval.py --checkpoint cpg_runs/cpg_rl_cpg0_standard/model_update_500.pt \
    --disturbance turbulent --intensity 1.0
```

## Troubleshooting

### CUDA Out of Memory

Reduce SPH particle count in `env/swim_gs_env.py`:

```python
sph_options=gs.options.SPHOptions(
    particle_size=0.04  # Increase from 0.03 (fewer particles)
)
```

### Slow Training

Use `short` profile or reduce batch steps:

```bash
python3 run.py --mode cpg_rl --profile short  # 100 updates
```

### Robot Unstable

Reduce disturbance intensity or modify flow coefficients in `fluid_disturbance.py`:

```python
# Line 136: Reduce constant current
delta_v += self.intensity * 0.15 * self.current_direction  # From 0.3 to 0.15
```

## Citation

If you use this code, please cite:

```bibtex
@software{cpg_rl_swimming,
  title = {CPG-RL Swimming Robot with Fluid Disturbance},
  author = {Your Name},
  year = {2025},
  url = {https://github.com/yourusername/cpg-swimming}
}
```

## License

MIT License - see LICENSE file for details.

## Acknowledgments

- Genesis physics simulator: https://genesis-world.readthedocs.io/
- PPO implementation based on Spinning Up (OpenAI)
