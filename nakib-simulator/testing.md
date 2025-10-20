# testing.py — Overview

This document explains the purpose and core functionality of `testing.py`.

## Purpose

`testing.py` provides headless simulation utilities and a test runner that reads CSV files of test configurations. It is used to evaluate how well a barrier-distance configuration keeps the car safe from obstacles over simulated frames.

## Key components

- Safety constants: `SAFE=0`, `MILD_UNSAFE=1`, `UNSAFE=2`.
- `TestResult` class: stores summarized results of a single test run (counts of frames per safety state, minimum distance observed, etc.).
- `TestDataCollector` class: accumulates frame-by-frame records for export to CSV and aggregates test results.
- `HeadlessCarSimulator` class: a stripped-down version of the simulator that calculates a path using the same planning logic and steps the `current_position` forward without rendering graphics.

### Important methods in `HeadlessCarSimulator`
- `calculate_safe_path()` — same waypoint generation approach as the animator but without plotting.
- `ensure_safe_point(point)` — ensures a point lies outside `barrier_distance` of all obstacles.
- `step()` — moves along the current segment according to `speed` and updates `current_position`; returns `False` when the end is reached.
- `get_distance_to_obstacles()` and `get_safety_state()` — compute proximity and safety categorization.

## CSV-driven testing

- `run_tests_from_csv(csv_file)` — reads a CSV where each row describes start/end, obstacle location, barrier distance, and speed. For each row, the function runs a headless test, collects frame-level data, and summarizes results.
- Results are saved into `test_results/summary_*.csv` and `test_results/detailed_*.csv` and visual plots are written into `test_results/plots/`.

## Notes

- This file is intended for automated experiments (sweeping barrier distances, obstacle configurations) and generating visual summaries.
- Plot legends and titles were converted to ASCII-only labels to avoid font glyph warnings when saving PNGs.
