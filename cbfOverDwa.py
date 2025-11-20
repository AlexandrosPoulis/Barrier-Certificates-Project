import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

alpha = 1.0
v_min, v_max = 0.0, 1.0
omega_min, omega_max = -1.2, 1.2

x_sym, y_sym = sp.symbols('x y')

B_expr = (
    0.42*x_sym**4 + 0.39*y_sym**4 + 0.15*x_sym**2*y_sym**2
    + 0.08*x_sym**3 + 0.06*y_sym**3
    + 0.07*x_sym**2 + 0.05*y_sym**2 + 0.03*x_sym*y_sym
    + 0.02*x_sym + 0.02*y_sym + 0.01
)

dBx_expr = sp.diff(B_expr, x_sym)
dBy_expr = sp.diff(B_expr, y_sym)

B_func = sp.lambdify((x_sym, y_sym), B_expr, 'numpy')
gradB_func = sp.lambdify((x_sym, y_sym), (dBx_expr, dBy_expr), 'numpy')

def B_xy(x, y):
    return B_func(x, y)

def gradB_xy(x, y):
    return gradB_func(x, y)

def cbf_filter (u_nom, state):
    x, y, theta, v_curr, omega_curr = state
    v_nom, omega_nom = u_nom

    Bv = B_xy (x, y)
    dBx, dBy = gradB_xy (x, y)

    c, s = np.cos(theta), np.sin(theta)
    coeff_v = (dBx * c + dBy * s)

    if abs (coeff_v) < 1e-9:
        v_req = v_max if (alpha * Bv) < 0 else v_nom
    else:
        v_req = max (v_min, min (v_max, (-alpha * Bv) / coeff_v))
    
    def satisfies (v):
        return (v * coeff_v + alpha * Bv) >= -1e-9
    
    candidates = [np.clip (v_nom, v_min, v_max), v_req]

    v_filt = None

    for vv in sorted (candidates, key = lambda z: abs (z - v_nom)):
        if satisfies (vv):
            v_filt = vv
            break
        if v_filt is None:
            v_filt = np.clip (v_req, v_min, v_max)

    omega_filt = np.clip (omega_nom, omega_min, omega_max)

    return v_filt, omega_filt


# Test cases:

# Safe:
# u_nom = (0.8, 0.5)
# state = (0.2, 0.3, np.pi/4, 0.7, 0.4)
# v_filt, omega_filt = cbf_filter (u_nom, state)
# print (v_filt)
# print (omega_filt)

# Unsafe:
# u_nom = (0.9, 0.0)
# u_nom = (0.38028032866118905, 0.0) # <- output
# state = (0.1, 0.1, np.pi, 0.7, 0.0)
# v_filt, omega_filt = cbf_filter (u_nom, state)
# print (v_filt)
# print (omega_filt)

# If the values of v_filt, omega_filt are different than u_nom, state; then its unsafe, signifying that the system was about to overlap with an unsafe region, so it slowed down.

def classify_case (u_nom, state, tol = 1e-4):
    v_nom, omega_nom = u_nom
    v_filt, omega_filt = cbf_filter (u_nom, state)

    safe = (
        abs (v_filt - v_nom) <= tol and
        abs (omega_filt - omega_nom) <= tol
    )

    results = {
        "x": state [0],
        "y": state [1],
        "theta": state [2],
        "v_curr": state [3],
        "omega_curr": state [4],
        "v_nom": v_nom,
        "omega_nom": omega_nom,
        "v_filt": v_filt,
        "omega_filt": omega_filt,
        "safe": safe
    }

    return results, safe

def sample_random_state ():
    x = np.random.uniform (-5.0, 5.0)
    y = np.random.uniform (-5.0, 5.0)
    theta = np.random.uniform (-np.pi, np.pi)
    v_curr = np.random.uniform (v_min, v_max)
    omega_curr = np.random.uniform (omega_min, omega_max)
    return (x, y, theta, v_curr, omega_curr)

def sample_random_control ():
    v_nom = np.random.uniform (v_min, v_max)
    omega_nom = np.random.uniform (omega_min, omega_max)
    return (v_nom, omega_nom)

def run_batch_tests (n_tests = 100):
    results = []
    safe_count = 0
    unsafe_count = 0

    for _ in range (n_tests):
        state = sample_random_state ()
        u_nom = sample_random_control ()
        res, safe = classify_case (u_nom, state)

        results.append (res)
        if safe:
            safe_count += 1
        else:
            unsafe_count += 1

    print (f"Total tests: {n_tests}")
    print (f"Safe cases: {safe_count}")
    print (f"Unsafe cases: {unsafe_count}")
    print (f"Safety ratio: {safe_count / n_tests:.2f}")

    return results

def save_results_to_txt (results, filename = "cbf_results.txt"):
    with open (filename, "w") as f:
        for i, r in enumerate (results):
            f.write (f"Case {i}: \n")
            for k, v in r.items ():
                f.write (f" {k}: {v}\n")
            f.write("\n")

if __name__ == "__main__":
    np.random.seed (0)

    print ("\nBatch testing...")
    results = run_batch_tests (n_tests = 200)
    save_results_to_txt (results, "cbf_batch_results.txt")

