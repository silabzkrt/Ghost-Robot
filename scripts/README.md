# Validation Scripts

This directory contains scripts for validating and testing the Ghost Robot implementation.

## validate_trajectory.py

Validates the mathematical correctness of the figure-8 trajectory algorithm.

### Usage

```bash
python3 scripts/validate_trajectory.py
```

### What it does

1. Simulates the figure-8 trajectory using the same parametric equations as the C++ code
2. Computes linear and angular velocities
3. Validates trajectory characteristics:
   - Symmetry around X and Y axes
   - Multiple center crossings (figure-8 characteristic)
   - Expected number of loops
4. Generates a visualization showing:
   - The figure-8 path
   - Linear velocity over time
   - Angular velocity over time
   - Velocity components (vx, vy)

### Requirements

```bash
pip install numpy matplotlib
```

### Output

The script generates a visualization saved to `/tmp/ghost_robot_trajectory.png` and prints validation statistics to the console.

### Example Output

```
Ghost Robot Figure-8 Trajectory Validation
==================================================

Trajectory Statistics:
  Duration: 19.98 seconds
  Number of points: 1000
  X range: [-1.000, 1.000]
  Y range: [-0.998, 0.998]
  Linear velocity range: [1.093, 3.512] m/s
  Angular velocity range: [-18.318, 18.318] rad/s

Figure-8 Validation:
  Center crossings: 20 (should be multiple)
  X-axis symmetry: ✓
  Y-axis symmetry: ✓
  Expected loops: 10.0

Validation complete! ✓
```

## Notes

- The validation uses the exact same mathematical formulas as the C++ implementation
- This ensures the C++ code will produce the correct figure-8 pattern when deployed
- The script is useful for understanding the trajectory behavior before running in Isaac Sim
