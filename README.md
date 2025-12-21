# Barrier-Certificates-Project

---

A git hub repo used to upload code writen and used for the CS-IT-07 semester PBL group project.

Any code in the legacy file are not used in the results of the report and may or may not be in working codition, any code outside of that folder is working and was used in the production of the final report.
The numpy_simulator_CBF.py file is the one mentioned on chapter 4.4 of the report, for no slack value used replace ρ with 0.

The rrtstar.py file is the code that was mentioned on chapter 4.1 of the report.

The overleaf link for the report is https://www.overleaf.com/read/vhcbfnnbwwzs#832fc3

---


## PRoTECT + DWA

#### Relevant files:

- **`bfc.py`**: produces barrier function `B(x)`, scalar values `gamma` and `lambda`, using PRoTECT, given:
  - state space,
  - an initial region,
  - one or more unsafe (obstacle) regions,
  - a dynamics vector `f`.

- **`cbfDwa.py`**: runs DWA with heuristic/standard-cbf filter applied that:
  - samples candidate velocities `(vx, vy)` from a dynamic window,
  - filters unsafe candidates using a safety check,
  - selects the best remaining candidate using a weighted cost,
  - simulates and plots trajectory + unsafe boxes + barrier.

---

###### Expected layout
```text
root/
  bfc.py
  cbfDwa.py
  PRoTECT/
````

###### Run the simulation

```bash
python cbfDwa.py
```

###### Run barrier synthesis only

```bash
python bfc.py
```

---

#### “Where do I edit X?”

###### 1) Obstacles (unsafe regions)

Inside `bfc.py`, within `compute_barrier(...)`,

define each obstacle as an **axis-aligned rectangle/square (box)** using two 2D arrays:

* `L_unsafei = [x_min, y_min]`  (lower-bound of the unsafe region)
* `U_unsafei = [x_max, y_max]`  (upper-bound of the unsafe region)

Example (one obstacle):

```python
L_unsafe1 = np.array([295.0, 300.0])
U_unsafe1 = np.array([305.0, 310.0])
```

In the current code, obstacles are assembled into:

* `L_unsafe = np.array([L_unsafe1, L_unsafe2, ...])`
* `U_unsafe = np.array([U_unsafe1, U_unsafe2, ...])`

###### Changing the number of obstacles

1. **In `bfc.py`**:

   * add/remove `L_unsafeK`, `U_unsafeK`,
   * update the stacking into `L_unsafe` / `U_unsafe`.

2. **In `cbfDwa.py` (plotting)**:

   * the current `plot_simulation_results(...)` is written for **exactly two** obstacles (indexes `[0]` and `[1]`).
   * to change the number of obstacles, the following must be updated accordingly to **loop** over all obstacles.

```python
for i, (L, U) in enumerate(zip(controller.region_data["L_unsafe"], controller.region_data["U_unsafe"]), start=1):
    plt.fill_between([L[0], U[0]], L[1], U[1], color="red", alpha=0.3, label=f"Unsafe region {i}")
```

---

###### 2) State space

Inside `bfc.py`, within `compute_barrier(...)`,

set:

* `L_space = [x_min, y_min]`
* `U_space = [x_max, y_max]`

`cbfDwa.py` also uses these bounds via `region_data` to reject trajectories that leave the workspace.

---

###### 3) Start / initial region

There are **two “start” concepts**:

###### (A) Initial region for PRoTECT (barrier synthesis)

Inside `bfc.py`, within `compute_barrier(...)`,

set the initial region:

* `L_initial = [x_min, y_min]`
* `U_initial = [x_max, y_max]`

###### (B) Simulation initial state (starting point of the system)

Inside `cbfDwa.py`, within `__main__`,

set:

```python
initial_state = np.array([x0, y0])
```

This point should be within the initial region defined in `bfc.py`.

---

###### 4) Goal

There are **two places** to keep consistent:

###### (A) Simulator's goal

Inside `cbfDwa.py`, within `DWACBFController.__init__`,

set:

```python
self.goal = np.array([gx, gy])
```

###### (B) Barrier connectivity sanity-check goal

Within `bfc.py`, inside `compute_barrier(...)`, when `_connectivity_check(...)` is called, it uses `goal = np.array([..., ...])`. If it determines that the goal is lies outside the calculated barrier, it will try finding another barrier with higher degree (checking all degree candidates defined within `candidate_degrees: Sequence[int] = (...)` until it finds a suitable canditate).

---

###### 5) Dynamics

There are two places where its defined:

###### (A) Simulator dynamics

Inside `cbfDwa.py`, within `def dynamics(self, state, v)`.

###### (B) PRoTECT dynamics

Inside `bfc.py`, within `compute_barrier(...)`, the vector field `f` (same dimension as `x`) contains the dynamics to input it to PRoTECT.

---

###### 6) Safety filter choice (heuristic vs standard cbf)

Inside `cbfDwa.py`, the active implementation is labeled:

* `# Heuristic Filter (Not CBF)` in `velocity_is_cbf_safe(...)`

The alternative is provided just below that part as commented code.
