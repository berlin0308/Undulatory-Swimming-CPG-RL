# CPG + RL Swimming Robot

Hybrid control for a swimming robot: **CPG (Central Pattern Generator)** and **RL (PPO)**. The simulator is built on **Genesis SPH** and is configured to run on **CUDA** and **macOS (Metal)** as well as other platforms.

<img width="1139" height="541" alt="image" src="https://github.com/user-attachments/assets/6ec5ad59-c521-44a8-b54c-a909d3ec052c" />


## Features

- **Hybrid Control**: CPG + PPO modulation
- **Paddle Disturbance (enabled)**: deterministic, controllable wave generator
- **Genesis SPH Simulation**: high-fidelity water simulation (backend auto-selected)


## Installation

### Prerequisites

- Python **3.11** recommended (Conda works well on macOS)
- Genesis: `genesis-world`

### Install dependencies

```bash
pip install -r requirements.txt
```

If you use Conda on macOS and want to be explicit:

```bash
/opt/anaconda3/bin/python3 -m pip install -r requirements.txt
```

## Quick Start

### Training

No paddle disturbance:

```bash
/opt/anaconda3/bin/python3 run.py --mode cpg_rl --cpg 0 --profile short
```

Enable paddle disturbance:

```bash
/opt/anaconda3/bin/python3 run.py --mode cpg_rl --cpg 0 --profile short \
  --paddle --paddle-count 2 --paddle-freq 0.2 --paddle-amp 0.17
```

### Key flags

- **`--mode`**: `direct` | `cpg` | `cpg_rl` | `res_rl`
- **`--cpg`**: `0` = PhaseOscillator (recommended), `1` = Matsuoka
- **`--paddle`**: enable paddle disturbance
- **`--paddle-count`**: number of paddles
- **`--paddle-freq`**: oscillation frequency (Hz)
- **`--paddle-amp`**: oscillation amplitude (m)


## Platform notes

### Genesis backend selection

The simulator auto-selects a safe backend (e.g., **Metal on macOS**). You can override it:

```bash
GENESIS_BACKEND=metal /opt/anaconda3/bin/python3 run.py --mode cpg_rl --profile short
```

Supported values: `cpu`, `metal`, `cuda` (if available).

## Project structure

```
.
├── run.py
├── eval.py
├── cpg_ppo_agent.py
├── cpg_env_adapter.py
├── cpg_config_utils.py
├── env/
│   ├── cpg.py
│   └── swim_gs_env.py
└── eelrobotv2_urdf/
    ├── urdf/
    └── meshes/
```

## Troubleshooting

### First run is slow

Genesis may compile kernels on the first run (especially on macOS/Metal). Let it finish; subsequent runs are faster.

### `wandb` not installed

This repo treats `wandb` as optional. If you want W&B logging:

```bash
pip install wandb
```


