
import os
import sys
import time
from typing import Tuple, Dict, Any, Optional, Sequence

import numpy as np
import sympy as sp

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTECT_ROOT = os.path.join(CURRENT_DIR, "PRoTECT")

if PROTECT_ROOT not in sys.path:
    sys.path.append(PROTECT_ROOT)

from PRoTECT.src.functions.ct_DS import ct_DS
from PRoTECT.src.functions.parallel_ct_DS import parallel_ct_DS


def _connectivity_check(
    B_func,
    lam: float,
    initial_center: np.ndarray,
    goal: np.ndarray,
    L_space: np.ndarray,
    U_space: np.ndarray,
    grid_res: float = 0.1,
) -> bool:

    x_vals = np.arange(L_space[0], U_space[0] + grid_res, grid_res)
    y_vals = np.arange(L_space[1], U_space[1] + grid_res, grid_res)
    X, Y = np.meshgrid(x_vals, y_vals, indexing="ij")
    B_grid = B_func(X, Y)

    safe = B_grid < lam
    idx_map = -np.ones_like(B_grid, dtype=int)

    def closest_idx(pt):
        i = int(round((pt[0] - L_space[0]) / grid_res))
        j = int(round((pt[1] - L_space[1]) / grid_res))
        i = np.clip(i, 0, safe.shape[0] - 1)
        j = np.clip(j, 0, safe.shape[1] - 1)
        return i, j

    start_ij = closest_idx(initial_center)
    goal_ij = closest_idx(goal)

    if not (safe[start_ij] and safe[goal_ij]):
        return False

    from collections import deque

    q = deque()
    q.append(start_ij)
    idx_map[start_ij] = 1

    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    while q:
        i, j = q.popleft()
        if (i, j) == goal_ij:
            return True
        for di, dj in dirs:
            ni, nj = i + di, j + dj
            if (
                0 <= ni < safe.shape[0]
                and 0 <= nj < safe.shape[1]
                and safe[ni, nj]
                and idx_map[ni, nj] < 0
            ):
                idx_map[ni, nj] = 1
                q.append((ni, nj))

    return False


def compute_barrier(
    dim: int = 2,
    use_parallel: bool = False,
    candidate_degrees: Sequence[int] = (2, 4, 6, 8),
    grid_resolution: float = 0.1,
):

    assert dim == 2, "Can't work for dim other than 2"

    L_initial = np.array([4.4, 4.4])
    U_initial = np.array([4.6, 4.6])

    L_unsafe1 = np.array([0.0, 4.0])
    U_unsafe1 = np.array([1.0, 6.0])

    L_unsafe2 = np.array([6.0, 6.0])
    U_unsafe2 = np.array([7.0, 8.0])

    L_unsafe = np.array([L_unsafe1, L_unsafe2])
    U_unsafe = np.array([U_unsafe1, U_unsafe2])

    L_space = np.array([0.0, 0.0])
    U_space = np.array([10.0, 10.0])

    x = sp.symbols(f"x1:{dim+1}")
    f = np.array([sp.Integer(0), sp.Integer(0)], dtype=object)

    fixed_params = {
        "dim": dim,
        "L_initial": L_initial,
        "U_initial": U_initial,
        "L_unsafe": L_unsafe,
        "U_unsafe": U_unsafe,
        "L_space": L_space,
        "U_space": U_space,
        "x": x,
        "f": f,
        "solver": "cvxopt",
        "gam": None,
        "lam": None,
        "l_degree": None,
    }

    best = None

    for deg in candidate_degrees:
        print(f"\nTrying PRoTECT barrier of degree {deg}")
        t0 = time.time()
        if use_parallel:
            result = parallel_ct_DS(deg, **fixed_params)
        else:
            result = ct_DS(deg, **fixed_params)
        t1 = time.time()
        print(f"PRoTECT (degree {deg}) elapsed time: {t1 - t0:.3f} s")

        if result is None or "barrier" not in result:
            print("  PRoTECT did not return a barrier for this degree.")
            continue

        B_expr = result["barrier"]
        gamma = float(result["gamma"])
        lam = float(result["lambda"])
        print(f"  gamma = {gamma}, lambda = {lam}")

        B_func = sp.lambdify((x[0], x[1]), B_expr, "numpy")
        initial_center = 0.5 * (L_initial + U_initial)
        goal = np.array([10.0, 10.0])

        connected = _connectivity_check(
            B_func, lam, initial_center, goal, L_space, U_space, grid_resolution
        )
        if connected:
            print(
                " Goal inside barrier "
            )
        else:
            print(
                " Goal outside barrier "
            )

        if best is None:
            best = (B_expr, gamma, lam)

    if best is None:
        raise RuntimeError("PRoTECT failed to produce any barrier.")

    barrier_expr, gamma, lam = best

    print(
        f"\nSelected barrier of degree {candidate_degrees[0]}: {barrier_expr}\n"
        f"  gamma = {gamma} , lambda = {lam}"
    )
    print(
        "WARNING: Global wall."
    )

    region_data = {
        "L_initial": L_initial,
        "U_initial": U_initial,
        "L_unsafe": L_unsafe,
        "U_unsafe": U_unsafe,
        "L_space": L_space,
        "U_space": U_space,
    }

    return barrier_expr, gamma, lam, region_data


if __name__ == "__main__":
    B, gamma, lam, region_data = compute_barrier(use_parallel=False)
    print("Barrier:", B)
    print("gamma:", gamma)
    print("lambda:", lam)
