# main.py — Overview

`main.py` provides an interactive command-line menu to run the simulator's main functionality.

## Functionality

- Offers a menu with options:
  1. Run Single Simulation — launches `animation.run_single_simulation()` which prompts for start/end and obstacles and runs the interactive animation.
  2. Run Tests from CSV File — calls `testing.run_tests_from_csv()` after letting the user choose a CSV file from `test_configs/` or elsewhere.
  3. Generate Test CSV File — launches the CSV generator from `data_generator.py` to create customized test sets.
  4. Exit

## Helper functions

- `select_file_manually()` and `get_csv_file()` help the user pick a CSV with tests.
- `clear_screen()` and `wait_for_user()` provide basic UX polish in a terminal.

## How to run

From `nakib-simulator` directory (after installing requirements):

```powershell
python main.py
```

Follow the menu prompts.
