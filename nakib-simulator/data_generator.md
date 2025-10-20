# data_generator.py — Overview

`data_generator.py` contains utilities for producing CSV test sets used by the test runner. These CSV files list test configurations: start/end coordinates, obstacle positions, barrier distances, speed, and descriptions.

## Main helper functions

- `create_sample_test_file()` — writes a small `test_configs/sample_tests.csv` demonstrating a few barrier distances and a single obstacle.
- `create_comprehensive_test_file()` — programmatically creates many test rows across multiple obstacle configurations and barrier distances and writes `test_configs/comprehensive_tests.csv`.
- `create_advanced_test_file()` — similar to the comprehensive set but focused on more complex obstacle patterns.
- `generate_test_csv()` — interactive helper that prompts the user for start/end points, obstacle coordinates, and barrier distance ranges and writes a custom CSV.

## CSV format

Each CSV contains the columns:

- `test_id, start_x, start_y, end_x, end_y, obstacle_x, obstacle_y, barrier_distance, speed, description`

These files are consumed by `testing.run_tests_from_csv()`.

## Usage

From the `main.py` menu, choose "Generate Test CSV File" to run the interactive generator, or run the helper functions directly from a Python prompt.
