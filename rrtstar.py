import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import math

# --- Safety Level Constants ---
SAFE = 0           # Vehicle is far from obstacles
MILD_UNSAFE = 1    # Vehicle is within caution zone
UNSAFE = 2         # Vehicle is dangerously close to obstacles

# ---------------------------
# Shared Utilities
# ---------------------------
class GeometryUtils:
    """Shared geometry calculations for distance, angles, and collision detection."""
    
    @staticmethod
    def init_obstacle_matrix(obstacle_positions):
        """Convert obstacle positions to NumPy matrix for efficient computation."""
        return np.array(obstacle_positions) if obstacle_positions else np.empty((0, 2))
    
    @staticmethod
    def calc_distance(p1, p2):
        """Calculate Euclidean distance between two points."""
        return np.linalg.norm(np.array(p2) - np.array(p1))
    
    @staticmethod
    def calc_distances_vectorized(point, points_matrix):
        """Calculate distances from one point to multiple points efficiently."""
        return np.linalg.norm(points_matrix - np.array(point), axis=1)
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to the range [-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    @staticmethod
    def has_obstacles(obs_matrix):
        """Check if the obstacle matrix contains any obstacles."""
        return len(obs_matrix) > 0


# ---------------------------
# 1. Path Planning Class (RRT* Algorithm)
# ---------------------------
class RRTNode:
    """Represents a node in the RRT* search tree."""
    def __init__(self, position, parent=None):
        self.position = np.array(position, dtype=float) # (x, y) coordinates
        self.parent = parent # reference to parent node
        self.cost = 0.0 # accumulated cost from start
        self.children = [] # list of child nodes


class RRTStarPlanner:
    """
    Rapidly-exploring Random Tree Star (RRT*) path planner.
    
    RRT* improves upon standard RRT by "rewiring" the tree: when a new node is added,
    it checks if existing nearby nodes can reach the start cheaper by going through 
    the new node. This leads to asymptotically optimal paths.
    """
    def __init__(self, start_point, end_point, obstacle_positions, barrier_distance):
        self.start = np.array(start_point)
        self.end = np.array(end_point)
        self.obs_matrix = GeometryUtils.init_obstacle_matrix(obstacle_positions)
        self.barrier_dist = barrier_distance
        
    def plan(self, max_iter=1000, step_size=0.2, goal_sample_rate=0.1):
        """
        Execute the RRT* planning algorithm.
        
        Args:
            max_iter: Maximum number of iterations
            step_size: Maximum distance to extend tree per iteration
            goal_sample_rate: Probability of sampling goal (0.0 to 1.0)
        """
        print(f"\n=== RRT* PATH PLANNING ===")
        
        start_node = RRTNode(self.start)
        goal_node = RRTNode(self.end)
        
        min_x, max_x, min_y, max_y = self._get_sampling_bounds()
        tree = [start_node]
        
        for iteration in range(max_iter):
            # Occasionally sample the goal directly to guide the tree
            if np.random.random() < goal_sample_rate:
                sample = self.end
            else:
                sample = np.array([np.random.uniform(min_x, max_x), 
                                 np.random.uniform(min_y, max_y)])
            
            # Find nearest node in the current tree to the sampled point.
            # This determines which branch we extend from.
            nearest = min(tree, key=lambda n: GeometryUtils.calc_distance(n.position, sample))
            
            # Steer from the nearest node toward the sample. The step size caps
            # how far we extend the tree in one iteration.
            new_pt = self._steer(nearest.position, sample, step_size)
            
            # collision check: only add the new node if the segment from the
            # nearest node to the new point does not intersect any obstacle
            if self._is_collision_free(nearest.position, new_pt):
                new_node = RRTNode(new_pt)
                
                # Find nearby nodes to consider for choosing a better parent
                # and for rewiring. The radius is heuristic and shrinks/grows
                # with tree size to balance exploration vs local optimization.
                nearby_radius = min(2.0, 2.0 * ((math.log(len(tree)) / len(tree)) ** (1.0/2.0)))
                nearby = [n for n in tree 
                         if GeometryUtils.calc_distance(n.position, new_pt) < nearby_radius]
                
                # Choose best parent
                best_parent = self._choose_parent(nearby, new_node, nearest)
                
                # Connect to tree
                new_node.parent = best_parent
                new_node.cost = best_parent.cost + GeometryUtils.calc_distance(
                    new_node.position, best_parent.position)
                # maintain bidirectional parent-child links
                best_parent.children.append(new_node)
                tree.append(new_node)
                
                # Rewire nearby nodes through the new node if it gives them
                # a cheaper path (this is the core optimization step in RRT*).
                self._rewire(tree, new_node, nearby)
                
                # Check if goal reached
                if GeometryUtils.calc_distance(new_node.position, self.end) < step_size:
                    if self._is_collision_free(new_node.position, self.end):
                        goal_node.parent = new_node
                        path = self._extract_path(goal_node)
                        print(f"Path found with {len(path)} waypoints.")
                        print("==========================\n")
                        return path
            
            if iteration % 200 == 0:
                print(f"RRT* Iteration {iteration}/{max_iter}...")

        # Fallback: return closest path
        print("Max iterations reached. Returning closest path.")
        best_node = min(tree, key=lambda n: GeometryUtils.calc_distance(n.position, self.end))
        return self._extract_path(best_node)

    def _get_sampling_bounds(self, margin=3.0):
        """Calculate bounds for random sampling."""
        if GeometryUtils.has_obstacles(self.obs_matrix):
            xs = np.concatenate(([self.start[0], self.end[0]], self.obs_matrix[:, 0]))
            ys = np.concatenate(([self.start[1], self.end[1]], self.obs_matrix[:, 1]))
        else:
            xs = [self.start[0], self.end[0]]
            ys = [self.start[1], self.end[1]]
        return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin

    def _steer(self, from_pt, to_pt, step):
        """Move from one point toward another by maximum step distance."""
        direction = to_pt - from_pt
        dist = GeometryUtils.calc_distance(from_pt, to_pt)
        
        if dist <= step:
            return to_pt
        return from_pt + (direction / dist) * step

    def _is_collision_free(self, p1, p2):
        """Check if line segment from p1 to p2 is collision-free."""
        if not GeometryUtils.has_obstacles(self.obs_matrix):
            return True
        
        # Distance between endpoints; used to determine sampling density
        dist = GeometryUtils.calc_distance(p1, p2)
        if dist < 1e-3:
            return True
        
        # Sample points along line segment
        # Create a few evenly-spaced sample points along the segment and
        # verify the distance from each sample to obstacles. This approximates
        # continuous collision checking while remaining efficient.
        num_checks = max(3, int(dist / 0.2))
        check_points = np.linspace(p1, p2, num_checks)

        for pt in check_points:
            if np.any(GeometryUtils.calc_distances_vectorized(pt, self.obs_matrix) < self.barrier_dist):
                # A sample lies within the safety barrier -> collision
                return False
        return True

    def _choose_parent(self, nearby, new_node, default):
        """Choose the best parent for new_node from nearby nodes."""
        best = default
        min_cost = default.cost + GeometryUtils.calc_distance(new_node.position, default.position)        
        for node in nearby:
            # Skip self references and any trivial cycles
            if node == new_node:
                continue
            # Only consider nodes that can connect collision-free to the
            # new node; compute the cost-to-come via that candidate parent.
            if self._is_collision_free(node.position, new_node.position):
                cost = node.cost + GeometryUtils.calc_distance(new_node.position, node.position)
                if cost < min_cost:
                    min_cost = cost
                    best = node
        return best

    def _rewire(self, tree, new_node, nearby):
        """Rewire tree to optimize paths through new_node."""
        for node in nearby:
            if node == new_node.parent:
                continue          
            dist = GeometryUtils.calc_distance(node.position, new_node.position)           
            # If going through new_node yields a lower cost and the connection
            # is collision-free, re-parent `node` to `new_node` and update costs.
            if (new_node.cost + dist < node.cost and 
                self._is_collision_free(new_node.position, node.position)):
                old_cost = node.cost
                # Maintain parent/child links consistently when moving nodes
                if node.parent:
                    node.parent.children.remove(node)
                new_node.children.append(node)
                node.parent = new_node
                node.cost = new_node.cost + dist
                # When a node's cost changes, all its descendants inherit the
                # cost delta; propagate that change down the subtree.
                self._update_children_costs(node, node.cost - old_cost)
    
    def _update_children_costs(self, node, cost_diff):
        """Recursively update costs of all descendant nodes after rewiring."""
        for child in node.children:
            child.cost += cost_diff
            self._update_children_costs(child, cost_diff)

    def _extract_path(self, end_node):
        """Extract and smooth path from start to end_node."""
        # Trace back through parents
        path = []
        curr = end_node
        while curr:
            path.append(curr.position)
            curr = curr.parent
        path = path[::-1]
        
        # Smooth path by removing unnecessary waypoints
        if len(path) <= 2:
            return path
                
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            for j in range(len(path) - 1, i, -1):
                if self._is_collision_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
                if i < len(path):
                    smoothed.append(path[i])
        return smoothed


# ---------------------------
# 2. Vehicle Controller (Pure Pursuit)
# ---------------------------
class VehicleController:
    """
    Controls vehicle motion using Pure Pursuit algorithm and bicycle kinematic model.
    """
    def __init__(self, start_pos, end_pos, path, obstacle_positions, barrier_dist, nominal_speed=1.2):
        # Kinematic state
        self.x, self.y = float(start_pos[0]), float(start_pos[1])
        
        dir_vec = end_pos - start_pos
        self.heading = math.atan2(dir_vec[1], dir_vec[0]) if np.linalg.norm(dir_vec) > 0 else 0.0
        
        self.v = 0.0
        self.steer_angle = 0.0
        
        # Path and environment
        self.path = path
        self.end_point = end_pos
        self.obs_matrix = GeometryUtils.init_obstacle_matrix(obstacle_positions)
        self.barrier_dist = barrier_dist
        self.current_idx = 0
        
        # Vehicle parameters
        self.wheelbase = 0.6
        self.max_steer = math.radians(30)
        self.max_steer_rate = math.radians(30)
        self.max_acc = 1.5
        self.max_dec = -3.0
        self.nominal_speed = nominal_speed
        
        self.has_reached_end = False

    @property
    def position(self):
        """Get current position as NumPy array."""
        return np.array([self.x, self.y])

    def get_state(self):
        """Get complete vehicle state (x, y, heading)."""
        return self.x, self.y, self.heading

    def check_safety(self):
        """Check safety status based on distance to nearest obstacle."""
        if not GeometryUtils.has_obstacles(self.obs_matrix):
            return SAFE, float('inf')
        
        dists = GeometryUtils.calc_distances_vectorized(self.position, self.obs_matrix)
        min_dist = np.min(dists)
        
        if min_dist < 0.5:
            return UNSAFE, min_dist
        elif min_dist < self.barrier_dist:
            return MILD_UNSAFE, min_dist
        return SAFE, min_dist

    def update(self, dt):
        """Update vehicle state for one time step."""
        if self.has_reached_end:
            return

        dist_to_end = GeometryUtils.calc_distance(self.position, self.end_point)

        # Pure Pursuit: Find lookahead point
        lookahead_dist = 0.8 + 0.6 * (self.v / max(1e-6, self.nominal_speed))
        target_pt, idx = self._find_lookahead(self.position, lookahead_dist)
        self.current_idx = idx
        
        # Calculate control outputs
        curvature = self._calc_curvature(target_pt, lookahead_dist)
        target_steer = math.atan(self.wheelbase * curvature)
        target_speed = self._calc_desired_speed(curvature, dist_to_end)
        
        # Apply steering rate limit
        max_dsteer = self.max_steer_rate * dt
        self.steer_angle += np.clip(target_steer - self.steer_angle, -max_dsteer, max_dsteer)
        self.steer_angle = np.clip(self.steer_angle, -self.max_steer, self.max_steer)
        
        # Apply acceleration limits
        speed_err = target_speed - self.v
        acc = self.max_acc if speed_err >= 0 else self.max_dec
        
        if abs(speed_err) > abs(acc * dt):
            self.v += acc * dt
        else:
            self.v = target_speed
        self.v = max(0.0, self.v)

        # Kinematic update (bicycle model)
        self.x += self.v * math.cos(self.heading) * dt
        self.y += self.v * math.sin(self.heading) * dt
        self.heading += (self.v / self.wheelbase) * math.tan(self.steer_angle) * dt
        self.heading = GeometryUtils.normalize_angle(self.heading)
        
        # Check arrival
        if dist_to_end < 0.15 and self.v < 0.05:
            self.has_reached_end = True

        # Notes on control flow:
        # - Lookahead distance grows with speed which smooths steering at higher
        #   velocities and shortens it when slow to allow finer corrections.
        # - Steering is rate-limited to model actuator limits and prevent
        #   unrealistic instantaneous wheel angle jumps.
        # - Acceleration uses symmetric limits (`max_acc` and `max_dec`) and
        #   is applied conservatively to avoid overshoot.

    def _find_lookahead(self, pos_vector, dist):
        """Find lookahead point on path."""
        for i in range(self.current_idx, len(self.path)):
            if GeometryUtils.calc_distance(self.path[i], pos_vector) >= dist:
                return self.path[i], i
        return self.path[-1], len(self.path) - 1

    def _calc_curvature(self, target, lookahead_dist):
        """Calculate path curvature using Pure Pursuit geometry."""
        dx = target[0] - self.x
        dy = target[1] - self.y
        
        cos_h = math.cos(-self.heading)
        sin_h = math.sin(-self.heading)
        
        local_x = cos_h * dx - sin_h * dy
        local_y = sin_h * dx + cos_h * dy
        
        if abs(local_x) < 1e-6:
            return 0.0
        
        # alpha is the angle to the lookahead point in the vehicle frame. The
        # Pure Pursuit curvature formula approximates the radius required to
        # reach that point and converts it into curvature (1/radius).
        alpha = math.atan2(local_y, local_x)
        return 2.0 * math.sin(alpha) / max(1e-6, lookahead_dist)

    def _calc_desired_speed(self, curvature, dist_to_end):
        """Calculate desired speed based on curvature and distance to goal."""
        if abs(curvature) < 1e-6:
            v_max_lat = self.nominal_speed
        else:
            radius = 1.0 / abs(curvature)
            v_max_lat = math.sqrt(2.0 * max(0.01, radius))
            
        desired = min(self.nominal_speed, v_max_lat)
        
        if dist_to_end < 1.0:
            desired = 0.3
        if dist_to_end < 0.15:
            desired = 0.0
            
        return desired

        # Speed selection notes:
        # - For sharp curves (high curvature, small radius) the lateral
        #   constraint reduces allowable speed to help maintain stability.
        # - As the vehicle approaches the goal the desired speed is reduced
        #   to allow smooth stopping.


# ---------------------------
# 3. Visualization Class
# ---------------------------
class CarVisualizer:
    """Handles visualization and animation of the simulation."""
    def __init__(self, sim_instance):
        self.sim = sim_instance
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.car_artists = []
        
        self.car_length = 0.8
        self.car_width = 0.4
        
        self.safe_frames = 0
        self.mild_frames = 0
        self.unsafe_frames = 0

    def setup(self):
        """Set up the plot with path, obstacles, and markers."""
        all_pts = [self.sim.start, self.sim.end] + self.sim.obstacles + list(self.sim.path)
        all_x, all_y = zip(*all_pts)
        
        self.ax.set_xlim(min(all_x) - 2, max(all_x) + 2)
        self.ax.set_ylim(min(all_y) - 2, max(all_y) + 2)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title("RRT* Path Planning Simulation")

        # Draw path
        path_arr = np.array(self.sim.path)
        self.ax.plot(path_arr[:, 0], path_arr[:, 1], 'b-', alpha=0.7, lw=2, label='Path')
        
        # Draw start and end
        self.ax.plot(*self.sim.start, 'go', markersize=10, label='Start')
        self.ax.plot(*self.sim.end, 'ro', markersize=10, label='End')
        
        # Draw obstacles
        for obs in self.sim.obstacles:
            self.ax.add_patch(Circle(obs, 0.5, color='red', alpha=0.3))
            self.ax.add_patch(Circle(obs, self.sim.barrier, color='orange', alpha=0.1, ls='--'))
        
        self.ax.legend()

    def _draw_car(self, x, y, heading, color):
        """Draw car as rectangle with wheels."""
        car = Rectangle((x - self.car_length/2, y - self.car_width/2),
                        self.car_length, self.car_width,
                        angle=np.degrees(heading),
                        rotation_point='center',
                        facecolor=color, edgecolor='black', zorder=5)
        
        patches = [car]
        
        # Add wheels: wheels are positioned relative to car center in local
        # coordinates (lx, ly) then rotated by the car heading to world coords.
        for wx, wy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            lx, ly = wx * (self.car_length/3.5), wy * (self.car_width/2)
            gx = lx * math.cos(heading) - ly * math.sin(heading)
            gy = lx * math.sin(heading) + ly * math.cos(heading)
            patches.append(Circle((x + gx, y + gy), 0.12, fc='black', zorder=6))
        
        return patches

    def update_frame(self, frame):
        """Animation update function."""
        self.sim.controller.update(self.sim.dt)
        
        for artist in self.car_artists:
            artist.remove()
        self.car_artists.clear()
        
        x, y, heading = self.sim.controller.get_state()
        safety, dist = self.sim.controller.check_safety()
        
        # Update statistics
        if safety == SAFE:
            self.safe_frames += 1
        elif safety == MILD_UNSAFE:
            self.mild_frames += 1
        else:
            self.unsafe_frames += 1
        
        # Color and status mapping
        color_map = {SAFE: '#00ccff', MILD_UNSAFE: '#ffaa00', UNSAFE: '#ff3300'}
        text_map = {SAFE: "SAFE", MILD_UNSAFE: "CAUTION", UNSAFE: "DANGER"}
        
        patches = self._draw_car(x, y, heading, color_map[safety])
        for p in patches:
            self.ax.add_patch(p)
            self.car_artists.append(p)
        
        speed = self.sim.controller.v * 3.6  # m/s to km/h cause we don't do miles here
        status = "ARRIVED" if self.sim.controller.has_reached_end else f"{speed:.1f} km/h"
        self.ax.set_title(f"Status: {status} | {text_map[safety]} | Min Dist: {dist:.2f} m")
        
        return self.car_artists

    def start_animation(self):
        """Begin animation."""
        self.setup()
        self.anim = FuncAnimation(self.fig, self.update_frame, interval=50, 
                                 blit=False, cache_frame_data=False)
        plt.show()


# ---------------------------
# 4. Main Simulation Class
# ---------------------------
class MovingCarSimulation:
    """Main simulation orchestrator."""
    def __init__(self, start, end, obstacles, barrier_dist=1.2, speed_scale=1.0):
        self.start = np.array(start)
        self.end = np.array(end)
        self.obstacles = obstacles
        self.barrier = barrier_dist
        self.dt = 0.1 * speed_scale
        
        # Path planning
        planner = RRTStarPlanner(start, end, obstacles, barrier_dist)
        self.path = planner.plan()
        
        # Vehicle controller
        self.controller = VehicleController(start, end, self.path, obstacles, barrier_dist)
        
        # Visualization
        self.viz = CarVisualizer(self)

    def run(self):
        """Run the simulation."""
        self.viz.start_animation()


# ---------------------------
# Change settings here, mainly to define start/end and obstacles
# ---------------------------
if __name__ == "__main__":
    # Define custom start, end, and obstacles
    start = np.array([3.0, 7.0])
    end = np.array([3.0, 0.0])
    
    # Custom obstacle positions (x, y)
    obstacles = [
        (4.0, 3.0),
        (2.8, 3.0),
    ]
    
    # Create and run simulation, change barrier distance and speed scale as needed
    sim = MovingCarSimulation(start, end, obstacles, barrier_dist=0.8, speed_scale=1.0)

    sim.run()
