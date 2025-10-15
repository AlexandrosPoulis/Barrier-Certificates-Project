import csv
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Safety states
SAFE = 0
MILD_UNSAFE = 1
UNSAFE = 2

class TestResult:
    def __init__(self, test_id, barrier_distance, total_frames, 
                 safe_frames, mild_unsafe_frames, unsafe_frames,
                 min_distance_observed, description=""):
        self.test_id = test_id
        self.barrier_distance = barrier_distance
        self.total_frames = total_frames
        self.safe_frames = safe_frames
        self.mild_unsafe_frames = mild_unsafe_frames
        self.unsafe_frames = unsafe_frames
        self.min_distance_observed = min_distance_observed
        self.description = description
        
    @property
    def safety_status(self):
        """Determine overall safety status based on frames"""
        if self.unsafe_frames > 0:
            return UNSAFE
        elif self.mild_unsafe_frames > 0:
            return MILD_UNSAFE
        else:
            return SAFE
    
    @property
    def safety_status_text(self):
        """Get text representation of safety status"""
        status = self.safety_status
        if status == SAFE:
            return "SAFE"
        elif status == MILD_UNSAFE:
            return "MILD_UNSAFE"
        else:
            return "UNSAFE"

class TestDataCollector:
    def __init__(self):
        self.test_results = []
        self.current_test_data = []
        
    def collect_frame_data(self, test_id, frame, position, distance_to_obstacle, safety_state):
        """Collect data for each frame during testing"""
        self.current_test_data.append({
            'test_id': test_id,
            'frame': frame,
            'position_x': position[0],
            'position_y': position[1],
            'distance_to_obstacle': distance_to_obstacle,
            'safety_state': safety_state,
            'safety_state_text': self._get_safety_state_text(safety_state)
        })
    
    def _get_safety_state_text(self, state):
        """Convert safety state to text"""
        if state == SAFE:
            return "SAFE"
        elif state == MILD_UNSAFE:
            return "MILD_UNSAFE"
        else:
            return "UNSAFE"
    
    def save_test_results(self, filename=None):
        """Save test results to CSV file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"test_results/summary_{timestamp}.csv"
        
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'test_id', 'barrier_distance', 'total_frames', 
                'safe_frames', 'mild_unsafe_frames', 'unsafe_frames',
                'safety_status', 'min_distance_observed', 'description'
            ])
            
            for result in self.test_results:
                writer.writerow([
                    result.test_id,
                    result.barrier_distance,
                    result.total_frames,
                    result.safe_frames,
                    result.mild_unsafe_frames,
                    result.unsafe_frames,
                    result.safety_status_text,
                    result.min_distance_observed,
                    result.description
                ])
        
        print(f"✅ Test results saved to {filename}")
        return filename
    
    def save_detailed_data(self, filename=None):
        """Save detailed frame-by-frame data"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"test_results/detailed_{timestamp}.csv"
        
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        df = pd.DataFrame(self.current_test_data)
        df.to_csv(filename, index=False)
        print(f"✅ Detailed data saved to {filename}")
        return filename

class HeadlessCarSimulator:
    """A headless version of the car simulator for testing without graphics"""
    
    def __init__(self, start_point, end_point, obstacle_positions, barrier_distance, speed=0.02):
        self.start_point = np.array(start_point, dtype=float)
        self.end_point = np.array(end_point, dtype=float)
        self.obstacle_positions = [np.array(obs, dtype=float) for obs in obstacle_positions]
        self.barrier_distance = barrier_distance
        self.speed = speed
        
        # Initialize current_position
        self.current_position = self.start_point.copy()
        
        # Calculate path that properly avoids obstacles
        self.path = self.calculate_safe_path()
        self.current_path_index = 0
        self.progress = 0.0
        self.has_reached_end = False
        
    def calculate_safe_path(self):
        """Calculate a safe path that avoids all obstacles with the given barrier distance"""
        if not self.obstacle_positions:
            return [self.start_point, self.end_point]
        
        path = [self.start_point]
        current_pos = self.start_point
        
        # Create waypoints that go around obstacles
        for obstacle in self.obstacle_positions:
            # Calculate vector from current position to obstacle
            to_obstacle = obstacle - current_pos
            obstacle_dist = np.linalg.norm(to_obstacle)
            
            # Calculate the main direction (start to end)
            main_direction = self.end_point - self.start_point
            if np.linalg.norm(main_direction) > 0:
                main_direction = main_direction / np.linalg.norm(main_direction)
            
            # Calculate perpendicular direction
            perp = np.array([-main_direction[1], main_direction[0]])
            
            # Check which side is better
            side1 = obstacle + perp * self.barrier_distance * 1.5
            side2 = obstacle - perp * self.barrier_distance * 1.5
            
            dist1 = np.linalg.norm(self.end_point - side1)
            dist2 = np.linalg.norm(self.end_point - side2)
            
            best_side = side1 if dist1 < dist2 else side2
            
            # Create approach and avoidance points
            safe_distance = self.barrier_distance * 1.2
            approach_point = obstacle - main_direction * safe_distance + perp * np.sign(np.dot(perp, best_side - obstacle)) * safe_distance
            
            # Ensure points are safe
            approach_point = self.ensure_safe_point(approach_point)
            avoidance_point = self.ensure_safe_point(best_side)
            
            # Add points to path
            if np.linalg.norm(approach_point - current_pos) > 0.1:
                path.append(approach_point)
                current_pos = approach_point
            
            if np.linalg.norm(avoidance_point - current_pos) > 0.1:
                path.append(avoidance_point)
                current_pos = avoidance_point
        
        # Add final destination
        final_point = self.ensure_safe_point(self.end_point)
        if np.linalg.norm(final_point - current_pos) > 0.1:
            path.append(final_point)
        
        # Remove duplicate points
        clean_path = []
        for point in path:
            if len(clean_path) == 0 or np.linalg.norm(point - clean_path[-1]) > 0.1:
                clean_path.append(point)
        
        return clean_path
    
    def ensure_safe_point(self, point):
        """Ensure a point is safe from all obstacles"""
        safe_point = point.copy()
        
        for obstacle in self.obstacle_positions:
            distance = np.linalg.norm(safe_point - obstacle)
            if distance < self.barrier_distance:
                direction_away = safe_point - obstacle
                if np.linalg.norm(direction_away) > 0:
                    direction_away = direction_away / np.linalg.norm(direction_away)
                safe_point = obstacle + direction_away * self.barrier_distance * 1.1
        
        return safe_point
    
    def step(self):
        """Perform one simulation step"""
        if self.has_reached_end:
            return False  # Simulation finished
        
        if self.current_path_index < len(self.path) - 1:
            segment_start = self.path[self.current_path_index]
            segment_end = self.path[self.current_path_index + 1]
            
            self.progress += self.speed
            
            if self.progress >= 1.0:
                self.progress = 0.0
                self.current_path_index += 1
                if self.current_path_index >= len(self.path) - 1:
                    self.has_reached_end = True
            
            if not self.has_reached_end:
                self.current_position = segment_start + self.progress * (segment_end - segment_start)
        
        return not self.has_reached_end
    
    def get_distance_to_obstacles(self):
        """Calculate minimum distance to any obstacle"""
        if not self.obstacle_positions:
            return float('inf')
        
        distances = [np.linalg.norm(self.current_position - obstacle) 
                    for obstacle in self.obstacle_positions]
        return min(distances)
    
    def get_safety_state(self):
        """Determine safety state based on distance to obstacles"""
        distance = self.get_distance_to_obstacles()
        
        # Define obstacle radius (considering car size)
        obstacle_radius = 0.5  # Car is approximately 0.8x0.4, so 0.5 is reasonable
        
        if distance < obstacle_radius:
            return UNSAFE  # Intersects obstacle
        elif distance < self.barrier_distance:
            return MILD_UNSAFE  # Intersects barrier circle but not obstacle
        else:
            return SAFE  # No intersection

def run_tests_from_csv(csv_file):
    """Run multiple tests from a CSV file with improved user flow"""
    print(f"🧪 Running tests from: {csv_file}")
    
    # Create data collector
    data_collector = TestDataCollector()
    
    try:
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            tests = list(reader)
        
        print(f"📊 Found {len(tests)} test configurations")
        print("⏳ Starting tests... This may take a while for large test sets.")
        
        for i, test in enumerate(tests):
            print(f"\n🔬 Running Test {i+1}/{len(tests)}")
            
            # Show test details
            description = test.get('description', 'No description')
            print(f"   Description: {description}")
            print(f"   Barrier Distance: {test['barrier_distance']}")
            print(f"   Obstacle: ({test['obstacle_x']}, {test['obstacle_y']})")
            
            # Parse test parameters
            start_point = (float(test['start_x']), float(test['start_y']))
            end_point = (float(test['end_x']), float(test['end_y']))
            obstacle_positions = [(float(test['obstacle_x']), float(test['obstacle_y']))]
            barrier_distance = float(test['barrier_distance'])
            speed = float(test.get('speed', 0.02))
            
            # Run simulation without animation but with data collection
            result = run_single_test(
                test_id=i+1,
                start_point=start_point,
                end_point=end_point,
                obstacle_positions=obstacle_positions,
                barrier_distance=barrier_distance,
                speed=speed,
                data_collector=data_collector,
                description=description
            )
            
            data_collector.test_results.append(result)
            
            # Show quick result with color coding
            if result.safety_status == SAFE:
                status = "✅ SAFE"
            elif result.safety_status == MILD_UNSAFE:
                status = "🟡 MILD_UNSAFE"
            else:
                status = "🔴 UNSAFE"
                
            print(f"   Result: {status}")
            print(f"   Frames - Safe: {result.safe_frames}, Mild Unsafe: {result.mild_unsafe_frames}, Unsafe: {result.unsafe_frames}")
            print(f"   Min Distance: {result.min_distance_observed:.3f}")
        
        # Save results
        summary_file = data_collector.save_test_results()
        detailed_file = data_collector.save_detailed_data()
        
        # Generate summary report
        generate_test_summary(data_collector.test_results)
        
        # Generate visualization
        generate_test_visualization(data_collector, summary_file, detailed_file)
        
        print(f"\n🎉 All tests completed!")
        print(f"📁 Results saved in: test_results/")
        
    except Exception as e:
        print(f"❌ Error running tests: {e}")
        import traceback
        traceback.print_exc()
        return

def run_single_test(test_id, start_point, end_point, obstacle_positions, barrier_distance, speed, data_collector, description=""):
    """Run a single test and collect data"""
    # Create headless car simulator
    car = HeadlessCarSimulator(start_point, end_point, obstacle_positions, barrier_distance, speed)
    
    safe_frames = 0
    mild_unsafe_frames = 0
    unsafe_frames = 0
    min_distance_observed = float('inf')
    frame_count = 0
    
    # Simulate movement without animation
    while car.step() and frame_count < 1000:  # Safety limit
        # Calculate distance to obstacle
        distance_to_obstacle = car.get_distance_to_obstacles()
        
        min_distance_observed = min(min_distance_observed, distance_to_obstacle)
        
        # Determine safety state
        safety_state = car.get_safety_state()
        
        # Count frames by safety state
        if safety_state == SAFE:
            safe_frames += 1
        elif safety_state == MILD_UNSAFE:
            mild_unsafe_frames += 1
        else:
            unsafe_frames += 1
        
        # Collect data
        data_collector.collect_frame_data(
            test_id, frame_count, car.current_position, 
            distance_to_obstacle, safety_state
        )
        
        frame_count += 1
    
    return TestResult(
        test_id=test_id,
        barrier_distance=barrier_distance,
        total_frames=frame_count,
        safe_frames=safe_frames,
        mild_unsafe_frames=mild_unsafe_frames,
        unsafe_frames=unsafe_frames,
        min_distance_observed=min_distance_observed,
        description=description
    )

def generate_test_summary(test_results):
    """Generate a summary report of test results"""
    print("\n" + "="*60)
    print("📈 TEST SUMMARY REPORT")
    print("="*60)
    
    # Group by description
    from collections import defaultdict
    grouped_results = defaultdict(list)
    
    for result in test_results:
        grouped_results[result.description].append(result)
    
    for description, results in grouped_results.items():
        print(f"\n📋 Configuration: {description}")
        
        safe_count = sum(1 for r in results if r.safety_status == SAFE)
        mild_unsafe_count = sum(1 for r in results if r.safety_status == MILD_UNSAFE)
        unsafe_count = sum(1 for r in results if r.safety_status == UNSAFE)
        total_count = len(results)
        
        print(f"   ✅ SAFE: {safe_count}/{total_count} ({safe_count/total_count*100:.1f}%)")
        print(f"   🟡 MILD_UNSAFE: {mild_unsafe_count}/{total_count} ({mild_unsafe_count/total_count*100:.1f}%)")
        print(f"   🔴 UNSAFE: {unsafe_count}/{total_count} ({unsafe_count/total_count*100:.1f}%)")
        
        # Find optimal barrier distance for this configuration
        safe_tests = [r for r in results if r.safety_status == SAFE]
        if safe_tests:
            optimal = min(safe_tests, key=lambda x: x.barrier_distance)
            print(f"   🎯 Optimal Barrier Distance: {optimal.barrier_distance}")
        else:
            # If no safe tests, find the one with least unsafe frames
            best_test = min(results, key=lambda x: x.unsafe_frames)
            print(f"   ⚠️  Best Barrier Distance: {best_test.barrier_distance} (Unsafe frames: {best_test.unsafe_frames})")
    
    # Overall statistics
    total_safe = sum(1 for r in test_results if r.safety_status == SAFE)
    total_mild_unsafe = sum(1 for r in test_results if r.safety_status == MILD_UNSAFE)
    total_unsafe = sum(1 for r in test_results if r.safety_status == UNSAFE)
    total_tests = len(test_results)
    
    print(f"\n📊 OVERALL STATISTICS:")
    print(f"   Total tests: {total_tests}")
    print(f"   ✅ SAFE: {total_safe} ({total_safe/total_tests*100:.1f}%)")
    print(f"   🟡 MILD_UNSAFE: {total_mild_unsafe} ({total_mild_unsafe/total_tests*100:.1f}%)")
    print(f"   🔴 UNSAFE: {total_unsafe} ({total_unsafe/total_tests*100:.1f}%)")
    
    # Find overall optimal barrier distance
    safe_tests = [r for r in test_results if r.safety_status == SAFE]
    if safe_tests:
        overall_optimal = min(safe_tests, key=lambda x: x.barrier_distance)
        print(f"   🏆 Overall Optimal Barrier Distance: {overall_optimal.barrier_distance}")

def generate_test_visualization(data_collector, summary_file, detailed_file):
    """Generate visualization plots for test results"""
    print("\n📊 Generating visualizations...")
    
    # Create plots directory
    os.makedirs("test_results/plots", exist_ok=True)
    
    if not data_collector.test_results:
        print("❌ No test results to visualize")
        return
    
    # Plot 1: Safety Status by Barrier Distance
    plt.figure(figsize=(12, 8))
    
    # Group by barrier distance
    from collections import defaultdict
    barrier_groups = defaultdict(list)
    
    for result in data_collector.test_results:
        barrier_groups[result.barrier_distance].append(result)
    
    barrier_distances = sorted(barrier_groups.keys())
    
    # Calculate percentages for each barrier distance
    safe_percentages = []
    mild_unsafe_percentages = []
    unsafe_percentages = []
    
    for barrier in barrier_distances:
        tests = barrier_groups[barrier]
        safe_count = sum(1 for t in tests if t.safety_status == SAFE)
        mild_unsafe_count = sum(1 for t in tests if t.safety_status == MILD_UNSAFE)
        unsafe_count = sum(1 for t in tests if t.safety_status == UNSAFE)
        total = len(tests)
        
        safe_percentages.append(safe_count/total * 100)
        mild_unsafe_percentages.append(mild_unsafe_count/total * 100)
        unsafe_percentages.append(unsafe_count/total * 100)
    
    # Stacked bar chart
    bar_width = 0.6
    x_pos = np.arange(len(barrier_distances))
    
    plt.bar(x_pos, safe_percentages, bar_width, label='SAFE', color='green', alpha=0.7)
    plt.bar(x_pos, mild_unsafe_percentages, bar_width, bottom=safe_percentages, 
        label='MILD_UNSAFE', color='orange', alpha=0.7)
    plt.bar(x_pos, unsafe_percentages, bar_width, 
        bottom=[safe_percentages[i] + mild_unsafe_percentages[i] for i in range(len(safe_percentages))],
        label='UNSAFE', color='red', alpha=0.7)
    
    plt.xlabel('Barrier Distance')
    plt.ylabel('Percentage (%)')
    plt.title('Safety Status Distribution by Barrier Distance')
    plt.xticks(x_pos, barrier_distances)
    plt.legend()
    plt.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('test_results/plots/safety_status_by_barrier.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 2: Min Distance vs Barrier Distance colored by safety status
    plt.figure(figsize=(10, 6))
    
    barrier_distances = [r.barrier_distance for r in data_collector.test_results]
    min_distances = [r.min_distance_observed for r in data_collector.test_results]
    safety_colors = []
    
    for result in data_collector.test_results:
        if result.safety_status == SAFE:
            safety_colors.append('green')
        elif result.safety_status == MILD_UNSAFE:
            safety_colors.append('orange')
        else:
            safety_colors.append('red')
    
    plt.scatter(barrier_distances, min_distances, c=safety_colors, s=60, alpha=0.7)
    plt.xlabel('Barrier Distance')
    plt.ylabel('Minimum Distance to Obstacle')
    plt.title('Minimum Distance vs Barrier Distance (Colored by Safety Status)')
    plt.grid(True, alpha=0.3)
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='green', label='SAFE'),
        Patch(facecolor='orange', label='MILD_UNSAFE'),
        Patch(facecolor='red', label='UNSAFE')
    ]
    plt.legend(handles=legend_elements)
    
    plt.tight_layout()
    plt.savefig('test_results/plots/min_distance_vs_barrier.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot 3: Frame Distribution by Safety State
    plt.figure(figsize=(10, 6))
    
    # Get a subset of tests for clarity
    sample_tests = data_collector.test_results[:10]  # Show first 10 tests
    
    test_ids = [f"Test {t.test_id}" for t in sample_tests]
    safe_frames = [t.safe_frames for t in sample_tests]
    mild_unsafe_frames = [t.mild_unsafe_frames for t in sample_tests]
    unsafe_frames = [t.unsafe_frames for t in sample_tests]
    
    x_pos = np.arange(len(test_ids))
    
    plt.bar(x_pos, safe_frames, bar_width, label='SAFE', color='green', alpha=0.7)
    plt.bar(x_pos, mild_unsafe_frames, bar_width, bottom=safe_frames, 
        label='MILD_UNSAFE', color='orange', alpha=0.7)
    plt.bar(x_pos, unsafe_frames, bar_width, 
        bottom=[safe_frames[i] + mild_unsafe_frames[i] for i in range(len(safe_frames))],
        label='UNSAFE', color='red', alpha=0.7)
    
    plt.xlabel('Test ID')
    plt.ylabel('Number of Frames')
    plt.title('Frame Distribution by Safety State (Sample Tests)')
    plt.xticks(x_pos, test_ids, rotation=45)
    plt.legend()
    plt.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('test_results/plots/frame_distribution.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("✅ Visualizations saved to test_results/plots/")
    print(f"📁 Summary file: {summary_file}")
    print(f"📁 Detailed data: {detailed_file}")

if __name__ == "__main__":
    # Create sample test file if it doesn't exist
    if not os.path.exists("test_configs/sample_tests.csv"):
        from data_generator import create_sample_test_file
        create_sample_test_file()
    
    run_tests_from_csv("test_configs/sample_tests.csv")