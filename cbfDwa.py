
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

from bfc import compute_barrier


class DWACBFController:
    def __init__(self):
    
        (
            self.barrier_expr,
            self.gamma,
            self.lam,
            self.region_data,
        ) = compute_barrier(dim=2, use_parallel=False)

        print(f"barrier: {self.barrier_expr}")
        print(f"gamma = {self.gamma}, lambda = {self.lam}")

        self.x1, self.x2 = sp.symbols("x1 x2")
        self.B_expr = sp.sympify(self.barrier_expr)
        self.dBdx1_expr = sp.diff(self.B_expr, self.x1)
        self.dBdx2_expr = sp.diff(self.B_expr, self.x2)

        self.B_func = sp.lambdify((self.x1, self.x2), self.B_expr, "numpy")
        self.gradB_func = sp.lambdify(
            (self.x1, self.x2), (self.dBdx1_expr, self.dBdx2_expr), "numpy"
        )

        self.L_unsafe = self.region_data["L_unsafe"]
        self.U_unsafe = self.region_data["U_unsafe"]

        self.v_max = 1.0
        self.a_max = 0.5
        self.v_resolution = 0.1

        self.dt = 0.1
        self.predict_time = 2.0
        self.N_pred = int(self.predict_time / self.dt)

        self.goal = np.array([10.0, 10.0])

        self.obstacle_margin = 0.2


    def B_xy(self, x1, x2):
        return float(self.B_func(x1, x2))

    def gradB_xy(self, x1, x2):
        dBdx1, dBdx2 = self.gradB_func(x1, x2)
        return float(dBdx1), float(dBdx2)


    def in_unsafe_box(self, x1, x2) -> bool:
        for (L, U) in zip(self.L_unsafe, self.U_unsafe):
            if L[0] <= x1 <= U[0] and L[1] <= x2 <= U[1]:
                return True
        return False

    def near_unsafe_box(self, x1, x2) -> bool:
        m = self.obstacle_margin
        for (L, U) in zip(self.L_unsafe, self.U_unsafe):
            if (L[0] - m) <= x1 <= (U[0] + m) and (L[1] - m) <= x2 <= (U[1] + m):
                return True
        return False

    def dynamics(self, state, v):

        x1, x2 = state
        vx, vy = v
        x1_next = x1 + vx * self.dt
        x2_next = x2 + vy * self.dt
        return np.array([x1_next, x2_next])

    def trajectory_predict(self, state, v):

        traj = [state.copy()]
        s = state.copy()
        for _ in range(self.N_pred):
            s = self.dynamics(s, v)
            traj.append(s.copy())
        return np.array(traj)


    def goal_cost(self, trajectory):
        final = trajectory[-1]
        return np.linalg.norm(final - self.goal)

    def barrier_cost(self, trajectory):

        cost = 0.0
        for (x1, x2) in trajectory:
            if self.near_unsafe_box(x1, x2):
                B_val = self.B_xy(x1, x2)
                cost = max(cost, max(0.0, B_val - self.lam))
        return cost

    def speed_cost(self, v):
        vx, vy = v
        speed = np.linalg.norm([vx, vy])
        return 1.0 - min(speed, self.v_max) / self.v_max

    def in_state_box(self, trajectory):
        L_space = self.region_data["L_space"]
        U_space = self.region_data["U_space"]
        for x1, x2 in trajectory:
            if not (L_space[0] <= x1 <= U_space[0] and L_space[1] <= x2 <= U_space[1]):
                return False
        return True

    def trajectory_is_safe(self, trajectory):

        for (x1, x2) in trajectory:
            if self.in_unsafe_box(x1, x2):
                B_val = self.B_xy(x1, x2)
                if B_val >= self.lam:
                    return False
        return True


    def velocity_is_cbf_safe(self, state, v):

        x1, x2 = state
        vx, vy = v

        B_val = self.B_xy(x1, x2)
        h_val = -B_val + self.lam
        
        dBdx1, dBdx2 = self.gradB_xy(x1, x2)
        dhdx1, dhdx2 = -dBdx1, -dBdx2
        
        gamma_cbf = 1.0
        
        lhs = dhdx1 * vx + dhdx2 * vy
        rhs = -gamma_cbf * h_val
        
        if lhs < rhs:
            return False
        
        return True

    def _dynamic_window(self, current_v):

        v_min = np.maximum(current_v - self.a_max * self.dt, -self.v_max)
        v_max = np.minimum(current_v + self.a_max * self.dt, self.v_max)

        vx_vals = np.arange(v_min[0], v_max[0] + 1e-6, self.v_resolution)
        vy_vals = np.arange(v_min[1], v_max[1] + 1e-6, self.v_resolution)
        return vx_vals, vy_vals

    def dwa_control(self, current_state, current_v):
        vx_vals, vy_vals = self._dynamic_window(current_v)

        best_v = np.array([0.0, 0.0])
        best_traj = None
        best_cost = np.inf
        feasible = 0

        for vx in vx_vals:
            for vy in vy_vals:
                v = np.array([vx, vy])

                if not self.velocity_is_cbf_safe(current_state, v):
                    continue

                traj = self.trajectory_predict(current_state, v)

                if not self.in_state_box(traj):
                    continue
                if not self.trajectory_is_safe(traj):
                    continue

                feasible += 1

                J_goal = self.goal_cost(traj)
                J_bar = self.barrier_cost(traj)
                J_speed = self.speed_cost(v)

                total_cost = 0.7 * J_goal + 0.2 * J_bar + 0.1 * J_speed

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_traj = traj

        if feasible == 0:
            print("WARNING: no feasible safe velocities; using v = 0.")
            return np.array([0.0, 0.0]), None

        return best_v, best_traj

    def run_simulation(self, initial_state, max_steps=200, goal_tol=1.0):
        state = np.array(initial_state, dtype=float)
        traj = [state.copy()]
        controls = []

        current_v = np.array([0.0, 0.0])

        for k in range(max_steps):
            v, _ = self.dwa_control(state, current_v)
            controls.append(v)
            current_v = v
            state = self.dynamics(state, v)
            traj.append(state.copy())

            if np.linalg.norm(state - self.goal) < goal_tol:
                print(f"Goal reached at step {k}")
                break

        return np.array(traj), np.array(controls)


def plot_simulation_results(controller, trajectory):
    L_unsafe1 = controller.region_data["L_unsafe"][0]
    U_unsafe1 = controller.region_data["U_unsafe"][0]
    L_unsafe2 = controller.region_data["L_unsafe"][1]
    U_unsafe2 = controller.region_data["U_unsafe"][1]

    plt.figure(figsize=(10, 8))

    plt.fill_between(
        [L_unsafe1[0], U_unsafe1[0]],
        L_unsafe1[1],
        U_unsafe1[1],
        color="red",
        alpha=0.3,
        label="Unsafe region 1",
    )
    plt.fill_between(
        [L_unsafe2[0], U_unsafe2[0]],
        L_unsafe2[1],
        U_unsafe2[1],
        color="red",
        alpha=0.3,
        label="Unsafe region 2",
    )

    x_traj = trajectory[:, 0]
    y_traj = trajectory[:, 1]
    plt.plot(x_traj, y_traj, "b-", label="Trajectory")
    plt.plot(x_traj[0], y_traj[0], "go", label="Start")
    plt.plot(x_traj[-1], y_traj[-1], "ro", label="End")
    plt.plot(
        controller.goal[0],
        controller.goal[1],
        "g*",
        markersize=12,
        label="Goal",
    )

    x_vals = np.linspace(
        controller.region_data["L_space"][0],
        controller.region_data["U_space"][0],
        80,
    )
    y_vals = np.linspace(
        controller.region_data["L_space"][1],
        controller.region_data["U_space"][1],
        80,
    )
    X, Y = np.meshgrid(x_vals, y_vals)
    B_vals = controller.B_func(X, Y)

    cs = plt.contour(
        X,
        Y,
        B_vals,
        levels=[controller.gamma, controller.lam],
        colors=["blue", "purple"],
        linestyles=["--", "-"],
    )
    plt.clabel(cs, inline=True, fontsize=8)

    plt.xlabel("x1")
    plt.ylabel("x2")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    controller = DWACBFController()

    initial_state = np.array([4.4, 4.4])
    trajectory, controls = controller.run_simulation(initial_state)

    print(f"Finished in {len(trajectory)} steps.")
    plot_simulation_results(controller, trajectory)
