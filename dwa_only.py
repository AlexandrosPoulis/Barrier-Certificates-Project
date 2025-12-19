# dwa_only.py

import numpy as np
import matplotlib.pyplot as plt


class DWAController:
    def __init__(self):
        self.L_initial = np.array([4.4, 4.4], dtype=float)
        self.U_initial = np.array([4.6, 4.6], dtype=float)

        self.L_unsafe = np.array([[0.0, 4.0], [6.0, 6.0]], dtype=float)
        self.U_unsafe = np.array([[1.0, 6.0], [7.0, 8.0]], dtype=float)

        self.L_space = np.array([0.0, 0.0], dtype=float)
        self.U_space = np.array([10.0, 10.0], dtype=float)

        self.goal = np.array([10.0, 10.0], dtype=float)

        self.v_max = 1.0
        self.a_max = 0.5
        self.v_resolution = 0.1

        self.dt = 0.1
        self.predict_time = 2.0
        self.N_pred = int(self.predict_time / self.dt)

        self.w_goal = 0.7
        self.w_clear = 0.2
        self.w_speed = 0.1

        self.clear_eps = 1e-3

    def in_unsafe_box(self, x1: float, x2: float) -> bool:
        for (L, U) in zip(self.L_unsafe, self.U_unsafe):
            if L[0] <= x1 <= U[0] and L[1] <= x2 <= U[1]:
                return True
        return False

    @staticmethod
    def _distance_point_to_axis_aligned_box(px: float, py: float, L: np.ndarray, U: np.ndarray) -> float:

        dx = 0.0
        if px < L[0]:
            dx = L[0] - px
        elif px > U[0]:
            dx = px - U[0]

        dy = 0.0
        if py < L[1]:
            dy = L[1] - py
        elif py > U[1]:
            dy = py - U[1]

        return float(np.hypot(dx, dy))

    def min_clearance_to_obstacles(self, trajectory: np.ndarray) -> float:
        min_d = float("inf")
        for x1, x2 in trajectory:
            for (L, U) in zip(self.L_unsafe, self.U_unsafe):
                d = self._distance_point_to_axis_aligned_box(float(x1), float(x2), L, U)
                if d < min_d:
                    min_d = d
                    if min_d <= 0.0:
                        return 0.0
        return float(min_d)

    def dynamics(self, state: np.ndarray, v: np.ndarray) -> np.ndarray:
        x1, x2 = state
        vx, vy = v
        return np.array([x1 + vx * self.dt, x2 + vy * self.dt], dtype=float)

    def trajectory_predict(self, state: np.ndarray, v: np.ndarray) -> np.ndarray:
        traj = [state.copy()]
        s = state.copy()
        for _ in range(self.N_pred):
            s = self.dynamics(s, v)
            traj.append(s.copy())
        return np.array(traj, dtype=float)

    def in_state_box(self, trajectory: np.ndarray) -> bool:
        for x1, x2 in trajectory:
            if not (self.L_space[0] <= x1 <= self.U_space[0] and self.L_space[1] <= x2 <= self.U_space[1]):
                return False
        return True

    def trajectory_is_safe(self, trajectory: np.ndarray) -> bool:
        for x1, x2 in trajectory:
            if self.in_unsafe_box(float(x1), float(x2)):
                return False
        return True

    def goal_cost(self, trajectory: np.ndarray) -> float:
        final = trajectory[-1]
        return float(np.linalg.norm(final - self.goal))

    def clearance_cost(self, trajectory: np.ndarray) -> float:
        dmin = self.min_clearance_to_obstacles(trajectory)
        return float(1.0 / (dmin + self.clear_eps))

    def speed_cost(self, v: np.ndarray) -> float:
        speed = float(np.linalg.norm(v))
        return float(1.0 - min(speed, self.v_max) / self.v_max)

    def _dynamic_window(self, current_v: np.ndarray):
        v_min = np.maximum(current_v - self.a_max * self.dt, -self.v_max)
        v_max = np.minimum(current_v + self.a_max * self.dt, self.v_max)

        vx_vals = np.arange(v_min[0], v_max[0] + 1e-6, self.v_resolution)
        vy_vals = np.arange(v_min[1], v_max[1] + 1e-6, self.v_resolution)
        return vx_vals, vy_vals

    def dwa_control(self, current_state: np.ndarray, current_v: np.ndarray):
        vx_vals, vy_vals = self._dynamic_window(current_v)

        best_v = np.array([0.0, 0.0], dtype=float)
        best_traj = None
        best_cost = float("inf")
        feasible = 0

        for vx in vx_vals:
            for vy in vy_vals:
                v = np.array([vx, vy], dtype=float)

                traj = self.trajectory_predict(current_state, v)

                if not self.in_state_box(traj):
                    continue
                if not self.trajectory_is_safe(traj):
                    continue

                feasible += 1

                J_goal = self.goal_cost(traj)
                J_clear = self.clearance_cost(traj)
                J_speed = self.speed_cost(v)

                total_cost = self.w_goal * J_goal + self.w_clear * J_clear + self.w_speed * J_speed

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_traj = traj

        if feasible == 0:
            return np.array([0.0, 0.0], dtype=float), None

        return best_v, best_traj

    def sample_initial_state(self, rng: np.random.Generator | None = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        return rng.uniform(self.L_initial, self.U_initial)

    def run_simulation(self, initial_state: np.ndarray, max_steps: int = 200, goal_tol: float = 1.0):
        state = np.array(initial_state, dtype=float)
        traj = [state.copy()]
        controls = []

        current_v = np.array([0.0, 0.0], dtype=float)

        for _ in range(max_steps):
            v, _ = self.dwa_control(state, current_v)
            controls.append(v)
            current_v = v
            state = self.dynamics(state, v)
            traj.append(state.copy())

            if np.linalg.norm(state - self.goal) < goal_tol:
                break

        return np.array(traj, dtype=float), np.array(controls, dtype=float)


def plot_simulation(controller: DWAController, trajectory: np.ndarray, title: str = "dwa_only"):
    plt.figure(figsize=(10, 8))

    for i, (L, U) in enumerate(zip(controller.L_unsafe, controller.U_unsafe), start=1):
        plt.fill_between([L[0], U[0]], L[1], U[1], alpha=0.3, label=f"Unsafe region {i}")

    x_traj = trajectory[:, 0]
    y_traj = trajectory[:, 1]
    plt.plot(x_traj, y_traj, "b-", label="Trajectory")
    plt.plot(x_traj[0], y_traj[0], "go", label="Start")
    plt.plot(x_traj[-1], y_traj[-1], "ro", label="End")
    plt.plot(controller.goal[0], controller.goal[1], "g*", markersize=12, label="Goal")

    plt.xlabel("x1")
    plt.ylabel("x2")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlim(controller.L_space[0], controller.U_space[0])
    plt.ylim(controller.L_space[1], controller.U_space[1])
    plt.show()


if __name__ == "__main__":
    ctrl = DWAController()

    initial_state = np.array([4.4, 4.4], dtype=float)
    traj, _ = ctrl.run_simulation(initial_state=initial_state, max_steps=200, goal_tol=1.0)

    print(f"Finished: {len(traj)} states (including start).")
    plot_simulation(ctrl, traj)
