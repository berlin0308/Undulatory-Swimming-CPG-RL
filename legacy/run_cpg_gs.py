#!/usr/bin/env python3
"""
Genesis-based swimming simulation with CPG control - Main execution script
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import json

# Add the current directory to the path to import env modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the GenesisSwimmingSimulation class from the env module
from env.swim_gs_env import GenesisSwimmingSimulation

def test_genesis_cpg(cpg_model_type=0, debug_mode=False):
    """
    Test CPG network with Genesis simulation.
    
    Args:
        cpg_model_type (int): 0 for phase oscillator, 1 for Matsuoka oscillator
        debug_mode (bool): Enable debug mode with larger particles and top-down view
    """
    print("CPG Implementation with Genesis")
    print("0: Phase Oscillator Network")
    print("1: Matsuoka Oscillator Network")
    print(f"Debug mode: {'ON' if debug_mode else 'OFF'}")
    
    # Create simulation with debug mode
    sim = GenesisSwimmingSimulation(
        cpg_model_type=cpg_model_type,
        use_gpu=False,
        record_video=True,  # Re-enable video recording
        debug_mode=debug_mode  # Enable debug mode
    )
    
    # Run simulation
    try:
        sim.run_simulation(duration=1.0, log_every=25)  # Longer duration to observe movement
        print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} simulation completed successfully!")
    except KeyboardInterrupt:
        print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} simulation interrupted by user.")
        print("Video and data already saved during interruption.")
    except Exception as e:
        print(f"Simulation error: {e}")
        print("Generating plots with available data...")
    
    # Skip plotting as requested
    print("Skipping plot generation as requested.")
    
    print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} test completed!")


if __name__ == "__main__":
    cpg_model_type = 0  # 0 = Phase Oscillator, 1 = Matsuoka
    debug_mode = True   # Set to True for debug mode
    test_genesis_cpg(cpg_model_type, debug_mode)