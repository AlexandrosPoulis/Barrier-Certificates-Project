import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Initial set
L_initial = np.array([0.1, 0.1])
U_initial = np.array([0.5, 0.5])

# Unsafe sets
L_unsafe1 = np.array([3, 3])
U_unsafe1 = np.array([6, 6])

L_unsafe2 = np.array([15, 10])
U_unsafe2 = np.array([25, 11])

# State space
L_space = np.array([0.1, 0.1])
U_space = np.array([50, 50])

fig, ax = plt.subplots(figsize=(10, 10))

# Plot state space
state_space = patches.Rectangle(L_space, *(U_space - L_space), linewidth=1, edgecolor='black', facecolor='none', label='State Space')
ax.add_patch(state_space)

# Plot initial set
initial_set = patches.Rectangle(L_initial, *(U_initial - L_initial), linewidth=1, edgecolor='green', facecolor='green', alpha=0.5, label='Initial Set')
ax.add_patch(initial_set)

# Plot unsafe sets
unsafe1 = patches.Rectangle(L_unsafe1, *(U_unsafe1 - L_unsafe1), linewidth=1, edgecolor='red', facecolor='red', alpha=0.5, label='Unsafe Set 1')
ax.add_patch(unsafe1)

unsafe2 = patches.Rectangle(L_unsafe2, *(U_unsafe2 - L_unsafe2), linewidth=1, edgecolor='red', facecolor='red', alpha=0.5, label='Unsafe Set 2')
ax.add_patch(unsafe2)

# Set plot limits
ax.set_xlim(L_space[0] - 1, U_space[0] + 1)
ax.set_ylim(L_space[1] - 1, U_space[1] + 1)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Initial Set, Unsafe Sets, and State Space')
ax.legend()
ax.grid(True)

plt.show()
