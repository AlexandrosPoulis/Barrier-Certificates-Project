## Activate venve via .\venv\Scripts\activate.ps1

# IMPORTS FROM INSTALLS
import time
from math import sin, cos, tan, radians
import sympy as sp
import numpy as np

# IMPORTS FROM TOOL
from PRoTECT.src.functions.parallel_ct_DS import parallel_ct_DS
from PRoTECT.src.functions.ct_DS import ct_DS
# ========================= Parameters =========================
if __name__ == '__main__':
    dim = 2  # dimension of state space

    # Initial set
    L_initial = np.array([0.1, 0.1])
    U_initial = np.array([0.5, 0.5])

    # Unsafe set
    L_unsafe1 = np.array([3, 3])
    U_unsafe1 = np.array([6, 6])
    
    L_unsafe2 = np.array([15, 10])
    U_unsafe2 = np.array([25, 11])

    # combine unsafe regions
    L_unsafe = np.array([L_unsafe1, L_unsafe2])
    U_unsafe = np.array([U_unsafe1, U_unsafe2])

    # State space
    L_space = np.array([0.1, 0.1])
    U_space = np.array([50, 50])

    # ========================= Symbolic Variables =========================
    x = sp.symbols(f'x1:{dim + 1}')  # Create x1, x2, ..., x_degree symbols not used
    # ========================= Dynamics =========================
    
    # non linear dynamocs
    
    lr = 1 # distance from center of gravity to rear axle
           # just have it as 1 for simplicity
    v = 4 # velocity
    b = radians(6) # slip angle arround 4 to 6 degrees
    #radians is how you calculate degrees
    ps = v/lr * tan(b) # ψ orientation of the body-fixed frame typically 6 o 10 degrees according to google
    
    
    f1 =  v * (cos(ps) - sin(ps) * tan(b))
    f2 =  v * (sin(ps) - cos(ps) * tan(b))
    # page 3 of Future-Focused Control Barrier Functions for Autonomous Vehicle
    # Control paper
    
    # Define the vector field
    f = np.array([f1, f2])

    fixed_params = {
        'dim': dim,
        'L_initial': L_initial,
        'U_initial': U_initial,
        'L_unsafe': L_unsafe,
        'U_unsafe': U_unsafe,
        'L_space': L_space,
        'U_space': U_space,
        'x': x,
        'f': f,
        'solver': "cvxopt",
        'gam': None,
        'lam': None,
        'l_degree': None,
        # Add other fixed parameters here
    }

    # List of degree values
    max_degree_value = 6
    single_degree_value = 2
    start = time.time()
    
    ### Uncomment this line to run the parallel implementation
    #result = parallel_ct_DS(max_degree_value, **fixed_params)
    
    ### Uncomment this line to run the serial implementation
    result = ct_DS(single_degree_value,**fixed_params)
    
    end = time.time()
    print("elapsed time:", end-start)
    if result == None:
        print("Results dictionary is empty.")
    else:
        print(result) # to receive just the BF do result["barrier"]