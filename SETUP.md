# Quick Setup Guide

## Installation

1. Install Genesis physics simulator:
```bash
pip install genesis-world
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Quick Test

Verify installation by running a short training:

```bash
python3 run.py --mode cpg_rl --cpg 0 --profile short
```

This will:
- Create environment with SPH fluid simulation
- Train for 100 updates (~10 minutes on GPU)
- Save models to `cpg_runs/cpg_rl_cpg0_short/`

## Basic Commands

### Train without disturbance
```bash
python3 run.py --mode cpg_rl --cpg 0 --profile standard
```

### Train with fluid disturbance
```bash
python3 run.py --mode cpg_rl --cpg 0 --profile standard \
    --fluid_disturbance mixed --disturbance_intensity 1.0
```

### Evaluate trained model
```bash
python3 eval.py --checkpoint cpg_runs/cpg_rl_cpg0_standard/model_update_500.pt \
    --episodes 10 --render
```

## File Overview

- `run.py` - Training script
- `eval.py` - Evaluation script
- `cpg_env_adapter.py` - RL environment wrapper
- `cpg_ppo_agent.py` - PPO algorithm implementation
- `fluid_disturbance.py` - SPH-based water current system
- `cpg_config_utils.py` - Training configurations
- `env/cpg.py` - CPG oscillator models
- `env/swim_gs_env.py` - Genesis simulation environment
- `urdf/eelrobotv2_urdf.urdf` - Robot model

## Troubleshooting

**CUDA out of memory**: Reduce SPH particles in `env/swim_gs_env.py` line 206:
```python
particle_size=0.04  # Increase from 0.03
```

**Too slow**: Use shorter profile:
```bash
python3 run.py --mode cpg_rl --profile short
```

**Robot unstable with disturbance**: Reduce intensity:
```bash
python3 run.py --mode cpg_rl --fluid_disturbance mixed --disturbance_intensity 0.5
```

## Expected Results

After standard training (500 updates, ~1 hour):
- Average return: 0.2-0.3
- Swimming speed: 0.3-0.5 m/s
- Stable forward motion

With disturbance (intensity=1.0):
- Average return: 0.1-0.2
- Swimming speed: 0.25-0.4 m/s
- Robust to currents

## Next Steps

See [README.md](README.md) for detailed documentation and advanced usage.
