# Nakib Simulator

A small Python simulator for a moving car that avoids circular obstacles using a simple barrier-distance-based planning approach.

## Overview

This project contains a headless tester and an animation for visualizing a car moving from a start point to an end point while keeping a safe distance (barrier distance) from circular obstacles. Each obstacle is represented as two layers:

- Object circle (physical obstacle) — radius 0.5 units
- Barrier circle — radius equal to the `barrier_distance` specified per test

Safety states used by the simulator:

- SAFE: car is outside the barrier circle
- MILD_UNSAFE: car is inside the barrier circle but not intersecting the object
- UNSAFE: car intersects the object circle

## Files of interest

- `main.py` — interactive menu to run a single simulation, run tests from CSV files, or generate new test CSV files.
- `animation.py` — visualization/animation of the moving car and obstacles using Matplotlib.
- `testing.py` — headless simulator, test-runner, result collection, and visualization generation.
- `data_generator.py` — utilities to create sample, comprehensive, or custom CSV test files.

## Requirements

- Python 3.8 or newer (3.8, 3.9, 3.10, 3.11 are recommended)
- The following Python packages:
  - numpy
  - matplotlib
  - pandas

## How to run

Open a terminal in the `nakib-simulator` folder (the directory that contains `main.py`) and activate your virtual environment (see above). Then run:

```
python main.py

```

`main.py` provides a simple menu:

1. Run Single Simulation — runs `animation.py` interactive simulation (asks for start/end, obstacles, barrier distance).
2. Run Tests from CSV File — runs `testing.py`'s `run_tests_from_csv()` on a selected CSV file (see `test_configs/` for sample files).
3. Generate Test CSV File — launches the `data_generator.generate_test_csv()` interactive CSV creator.
4. Exit

### Running tests non-interactively

You can also import and call functions directly from `testing.py` in scripts or in an interactive Python session. Example (headless):

```python
from testing import run_tests_from_csv
run_tests_from_csv('test_configs/sample_tests.csv')
```

## Outputs

- `test_results/summary_*.csv` — summarized test results
- `test_results/detailed_*.csv` — frame-by-frame data
- `test_results/plots/` — generated PNG visualizations

## Notes and troubleshooting

- The project intentionally avoids using emoji in plot labels to prevent font glyph warnings when Matplotlib saves figures. If you see warnings about missing glyphs, ensure your font config supports the requested characters or remove non-ASCII characters from titles/labels.
- If plots look clipped in saved PNG files, try calling `plt.tight_layout()` (already used in the project) or increasing figure sizes/dpi.
- If you get an import error, ensure your virtual environment is active and packages are installed in that environment.

## Next steps / improvements

- Add automated unit tests for core path-planning routines.
- Expose obstacle radius and car size as configuration parameters.
- Add more sophisticated path planning and collision checking.