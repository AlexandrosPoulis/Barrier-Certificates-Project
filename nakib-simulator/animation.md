# animation.py — Detailed Report

This document explains the design, algorithms, and implementation details of `animation.py`. It focuses on how the simulator computes a path that avoids circular obstacles, how it creates waypoints, and how the Matplotlib-based animation is constructed and updated.

## Purpose of `animation.py`

`animation.py` provides an interactive, visual simulation of a car moving from a start point to an end point while avoiding circular obstacles. It computes a planned path using simple geometric reasoning (offsets around obstacles), renders obstacles and the car using Matplotlib, and animates the car along the planned path.

## Python packages used and key functions

- numpy
  - `numpy.array`, `numpy.linalg.norm`, vector arithmetic — used for coordinate math, direction vectors, distances.
  - `np.allclose` — used for comparing waypoints when removing duplicates.

- matplotlib
  - `matplotlib.pyplot` (`plt`) — plotting and figure management (`plt.subplots`, `plt.show`, `plt.savefig`, `plt.close`).
  - `matplotlib.animation.FuncAnimation` — drives the animation loop by repeatedly calling the `update()` function.
  - `matplotlib.patches.Rectangle`, `matplotlib.patches.Circle` — used to draw the car body, wheels, and obstacle circles on the axis.

- math
  - `math.atan2`, `math.cos`, `math.sin`, `math.degrees` — used for angle computations and rotating wheel positions and rectangles.

The module relies on standard Python features for control flow and logging (print statements used throughout to trace planning and progress).

## High-level class structure

The main class is `MovingCar`. Key attributes and methods:

- Attributes:
  - `start_point`, `end_point` — 2D coordinates for start and destination.
  - `obstacle_positions` — list of obstacles (each a 2D point).
  - `barrier_distance` — desired minimum distance to obstacle centers (barrier radius).
  - `path` — a list of 2D waypoints computed by `calculate_safe_path()`.
  - `current_path_index`, `progress` — indices and fractional progress used to move along segments.
  - Plotting objects: `fig`, `ax` (Matplotlib figure and axes).

- Methods (high level):
  - `calculate_safe_path()` — computes a sequence of waypoints that bypass each obstacle.
  - `ensure_safe_point(point)` — moves a point away from obstacles if it falls inside any barrier circle.
  - `setup_plot()` — configures the axis, draws obstacles and path, adds initial patches.
  - `create_car_patches(position)` — constructs Matplotlib patches (rectangle + wheels) representing the car at a given position and orientation.
  - `calculate_car_angle()` — returns the heading angle of the car based on the current path segment.
  - `update(frame)` — the animation update invoked by `FuncAnimation` on each frame. Moves car along path, updates safety counters, replaces the car patches, and updates the title.
  - `animate()` — initializes animation and calls `plt.show()`.

## Path planning algorithm (detailed)

The goal is to produce a list of waypoints that take the car from `start_point` to `end_point` while keeping it outside the `barrier_distance` circle around each obstacle. The algorithm is simple, deterministic, and intended for clarity over optimality.

For each obstacle (in the order given):

1. Compute the main direction unit vector from the `start_point` to `end_point`:
   - `main_direction = (end_point - start_point) / ||end_point - start_point||` (if the norm is non-zero).

2. Compute a perpendicular vector `perp = [-main_direction[1], main_direction[0]]`. This yields a vector orthogonal to the path direction; moving along `perp` offsets side-to-side.

3. Generate two candidate avoidance side points at a scaled barrier distance:
   - `side1 = obstacle + perp * barrier_distance * 1.5`
   - `side2 = obstacle - perp * barrier_distance * 1.5`
   The factor (1.5) gives a buffer to avoid skimming the barrier.

4. Choose which side is better by measuring which of `side1` or `side2` yields a shorter remaining distance to the `end_point`.
   - Compute `dist1 = ||end_point - side1||` and `dist2 = ||end_point - side2||`.
   - `best_side = side1 if dist1 < dist2 else side2`

5. Compute an approach point (a point before the obstacle) to arrive at the avoidance side smoothly:
   - `safe_distance = barrier_distance * 1.2`
   - `approach_point = obstacle - main_direction * safe_distance + perp * sign(dot(perp, best_side - obstacle)) * safe_distance`
   This takes the obstacle center, moves backward along the path direction by `safe_distance` (so the car approaches from a bit before the obstacle), and offsets sideways along `perp` in the same sign direction as the best side.

6. Run both `approach_point` and `best_side` through `ensure_safe_point()` to make sure they are not inside any barrier circle; if they are, `ensure_safe_point()` pushes them outward along the direction away from the nearest obstacle center until they lie outside the barrier.

7. Append `approach_point`, then `avoidance_point` (the adjusted `best_side`), to the current path if they differ significantly from the current point (threshold ~0.1 units) to avoid adding near-duplicates.

8. After processing all obstacles, add the final destination (ensuring it's safe) and remove duplicate points (using `np.allclose` or a distance threshold).

Notes about the planner:
- It's greedy and sequential: handles obstacles one-by-one in the provided order.
- It does not perform global optimization and may fail to find a path in very congested scenarios, but it produces human-readable, deterministic maneuvers that work well for sparse obstacles.
- `ensure_safe_point()` is the post-processor that ensures waypoints are outside barriers.

Edge cases handled:
- If there are no obstacles, path is simply `[start_point, end_point]`.
- Small duplicate/nearby waypoints are collapsed.
- If `main_direction` length is zero (start == end), the algorithm avoids division by zero.

## Waypoint creation and path representation

- The `path` is a list of 2D numpy arrays representing positions the car should pass through in sequence.
- Movement between consecutive path points is linear interpolation: the car moves from `path[i]` to `path[i+1]` using a `progress` scalar that increases by `speed` each animation step: `position = start + progress * (end - start)`.
- When `progress >= 1.0`, the simulator sets `progress = 0.0`, increments `current_path_index`, and begins traversing the next segment.

## Animation internals

- `FuncAnimation(fig, update, ...)` calls `update(frame)` repeatedly. The code uses `blit=False` (full redraw per frame), which is simpler and robust given patches are added/removed dynamically.

- The visual representation per frame:
  1. Remove old car patches selectively (the code checks patch colors to avoid removing static scene elements like obstacle patches).
  2. Advance `progress` along the current segment by `speed`.
  3. Compute the new `current_position` as linear interpolation between segment endpoints.
  4. Update safety counters by calling `get_distance_to_obstacles()` and `get_safety_state()`.
  5. Build new car patches with `create_car_patches(position)`:
     - `create_car_patches` draws a rotated rectangle for the car body and four small circles for wheels. Rotation is derived from `calculate_car_angle()` which uses `atan2` on the direction to the next waypoint.
  6. Set the car body color based on safety state (e.g., blue/orange/red) and add the patches to the axes.
  7. Update the plot title with current position, safety state, and min distance observed.

- At the end of the animation (upon `plt.show()` returning), the script prints a final safety summary showing counts of frames categorized as SAFE / MILD_UNSAFE / UNSAFE and the minimum distance observed.

Implementation notes and suggestions

- The current animator uses full redraws each frame. For better performance on larger scenes or higher frame rates, consider:
  - Using `blit=True` and managing persistent artists so only changing patches are updated.
  - Pre-creating obstacle patches and path lines once in `setup_plot()` (already done for obstacles) and only updating car patches.

- The path planner is intentionally simple. For more robust behavior in cluttered environments, consider:
  - Replacing the naive waypoint generator with an established planner (A*, RRT, visibility graph, or PRM) on a discretized configuration space.
  - Adding turning radius constraints and kinematic bicycle model if vehicle dynamics matter.

## How to run the animation

Activate the virtual environment and run `main.py`, choose `Run Single Simulation` from the menu. Alternately, import `MovingCar` into a REPL and call `MovingCar(...).animate()`.

Example (headless quick run — not animated):
```python
from animation import MovingCar
car = MovingCar((0,0), (20,20), [(15,15)], barrier_distance=1.5, speed=0.02)
car.animate()  # interactive plot
```

## Conclusion

`animation.py` is a readable, well-instrumented animation that demonstrates barrier-distance based avoidance with clear debug prints. The path planning uses geometric offsets and ensures waypoints respect barrier distances. The animation leverages Matplotlib patches and `FuncAnimation` to produce an informative visualization. The module is suitable for demos and quick experimentation; for production-ready robust navigation, swap the planner for an algorithm that reasons globally about multiple obstacles at once.
