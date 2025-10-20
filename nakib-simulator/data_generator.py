import csv
import os

def create_sample_test_file():
    """Create a sample test file for demonstration"""
    sample_data = [
        [1, 0, 0, 10, 8, 5, 4, 1.0, 0.02, "Single obstacle at (5,4)"],
        [2, 0, 0, 10, 8, 5, 4, 1.5, 0.02, "Single obstacle at (5,4)"],
        [3, 0, 0, 10, 8, 5, 4, 2.0, 0.02, "Single obstacle at (5,4)"],
        [4, 0, 0, 10, 8, 5, 4, 2.5, 0.02, "Single obstacle at (5,4)"],
        [5, 0, 0, 10, 8, 5, 4, 3.0, 0.02, "Single obstacle at (5,4)"]
    ]
    
    os.makedirs("test_configs", exist_ok=True)
    
    with open("test_configs/sample_tests.csv", 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['test_id', 'start_x', 'start_y', 'end_x', 'end_y', 
                        'obstacle_x', 'obstacle_y', 'barrier_distance', 'speed', 'description'])
        writer.writerows(sample_data)
    
    print("✅ Created sample test file: test_configs/sample_tests.csv")

def create_comprehensive_test_file():
    """Create a comprehensive test file with varying obstacles and barrier distances"""
    test_cases = []
    test_id = 1
    
    # Base configuration - from (0,0) to (20,20)
    start_point = (0, 0)
    end_point = (20, 20)
    speed = 0.02
    
    # Different obstacle configurations (obstacles only after x=10)
    obstacle_configs = [
        {
            "name": "1_obstacle_after_10",
            "obstacles": [(15, 15)]
        },
        {
            "name": "2_obstacles_after_10",
            "obstacles": [(12, 12), (17, 17)]
        },
        {
            "name": "3_obstacles_after_10",
            "obstacles": [(11, 11), (15, 15), (18, 18)]
        },
        {
            "name": "clustered_obstacles_after_10",
            "obstacles": [(13, 13), (14, 14), (15, 15)]
        },
        {
            "name": "diagonal_obstacles_after_10",
            "obstacles": [(12, 8), (15, 15), (18, 12)]
        },
        {
            "name": "dense_obstacles_after_10",
            "obstacles": [(11, 11), (12, 12), (13, 13), (14, 14), (15, 15)]
        },
        {
            "name": "scattered_obstacles_after_10",
            "obstacles": [(11, 9), (13, 16), (15, 11), (17, 18), (19, 13)]
        },
        {
            "name": "narrow_path_obstacles",
            "obstacles": [(12, 10), (12, 14), (16, 10), (16, 14)]
        }
    ]
    
    # Barrier distances to test - focus on critical range for three-state analysis
    barrier_distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
    
    # Generate test cases
    for config in obstacle_configs:
        for barrier_distance in barrier_distances:
            # Create one test for each obstacle in the configuration
            for obstacle in config["obstacles"]:
                test_cases.append([
                    test_id,
                    start_point[0], start_point[1],
                    end_point[0], end_point[1],
                    obstacle[0], obstacle[1],
                    barrier_distance,
                    speed,
                    config["name"]
                ])
                test_id += 1
    
    # Write to CSV
    os.makedirs("test_configs", exist_ok=True)
    
    with open("test_configs/comprehensive_tests.csv", 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            'test_id', 'start_x', 'start_y', 'end_x', 'end_y', 
            'obstacle_x', 'obstacle_y', 'barrier_distance', 'speed', 'description'
        ])
        writer.writerows(test_cases)
    
    print(f"✅ Created comprehensive test file with {len(test_cases)} test cases")
    print("📊 Test variations:")
    print(f"   - Obstacle configurations: {len(obstacle_configs)}")
    print(f"   - Barrier distances: {len(barrier_distances)}")
    print(f"   - Total tests: {len(test_cases)}")
    print(f"   - Path: {start_point} to {end_point}")
    print(f"   - Obstacles placed only after x=10")
    print(f"   - Three-state safety analysis: SAFE, MILD_UNSAFE, UNSAFE")
    
    return "test_configs/comprehensive_tests.csv"

# ... (rest of data_generator.py remains the same)
def create_advanced_test_file():
    """Create advanced test file with specific obstacle patterns"""
    test_cases = []
    test_id = 1
    
    # Base configuration
    start_point = (0, 0)
    end_point = (20, 20)
    speed = 0.02
    
    # Advanced obstacle configurations
    obstacle_configs = [
        {
            "name": "single_center_obstacle",
            "obstacles": [(15, 15)]
        },
        {
            "name": "double_symmetric_obstacles",
            "obstacles": [(12, 12), (17, 17)]
        },
        {
            "name": "triangular_obstacles",
            "obstacles": [(11, 11), (15, 15), (18, 18)]
        },
        {
            "name": "wall_of_obstacles",
            "obstacles": [(13, 10), (13, 12), (13, 14), (13, 16), (13, 18)]
        },
        {
            "name": "checkerboard_obstacles",
            "obstacles": [(12, 12), (12, 16), (16, 12), (16, 16)]
        }
    ]
    
    # Focus on critical barrier distances
    barrier_distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    
    # Generate test cases
    for config in obstacle_configs:
        for barrier_distance in barrier_distances:
            for obstacle in config["obstacles"]:
                test_cases.append([
                    test_id,
                    start_point[0], start_point[1],
                    end_point[0], end_point[1],
                    obstacle[0], obstacle[1],
                    barrier_distance,
                    speed,
                    config["name"]
                ])
                test_id += 1
    
    # Write to CSV
    os.makedirs("test_configs", exist_ok=True)
    
    with open("test_configs/advanced_tests.csv", 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            'test_id', 'start_x', 'start_y', 'end_x', 'end_y', 
            'obstacle_x', 'obstacle_y', 'barrier_distance', 'speed', 'description'
        ])
        writer.writerows(test_cases)
    
    print(f"✅ Created advanced test file with {len(test_cases)} test cases")
    return "test_configs/advanced_tests.csv"

def generate_test_csv():
    """Generate a custom test CSV file based on user input"""
    print("\n📝 Generate Custom Test CSV File")
    print("=" * 40)
    
    # Get base configuration from user
    print("Enter base configuration:")
    start_x = float(input("Start X (default 0): ") or "0")
    start_y = float(input("Start Y (default 0): ") or "0")
    end_x = float(input("End X (default 20): ") or "20")
    end_y = float(input("End Y (default 20): ") or "20")
    
    # Get obstacle configuration
    print("\nEnter obstacle configuration:")
    num_obstacles = int(input("Number of obstacles (default 3): ") or "3")
    
    obstacles = []
    for i in range(num_obstacles):
        print(f"Obstacle {i+1}:")
        obs_x = float(input(f"  X coordinate (default {10 + i*2}): ") or str(10 + i*2))
        obs_y = float(input(f"  Y coordinate (default {10 + i*2}): ") or str(10 + i*2))
        obstacles.append((obs_x, obs_y))
    
    speed = float(input("\nSpeed (default 0.02): ") or "0.02")
    
    # Get barrier distance range
    print("\nEnter barrier distance range:")
    min_barrier = float(input("Minimum barrier distance (default 0.5): ") or "0.5")
    max_barrier = float(input("Maximum barrier distance (default 3.0): ") or "3.0")
    step = float(input("Step size (default 0.5): ") or "0.5")
    
    # Generate barrier distances
    barrier_distances = []
    current = min_barrier
    while current <= max_barrier:
        barrier_distances.append(round(current, 2))
        current += step
    
    # Get filename
    filename = input("\nOutput filename (default: test_configs/custom_tests.csv): ") 
    if not filename:
        filename = "test_configs/custom_tests.csv"
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    # Generate test cases
    test_cases = []
    test_id = 1
    
    for barrier_distance in barrier_distances:
        for obstacle in obstacles:
            test_cases.append([
                test_id,
                start_x, start_y,
                end_x, end_y,
                obstacle[0], obstacle[1],
                barrier_distance,
                speed,
                f"Custom_test_{test_id}"
            ])
            test_id += 1
    
    # Write to CSV
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([
            'test_id', 'start_x', 'start_y', 'end_x', 'end_y', 
            'obstacle_x', 'obstacle_y', 'barrier_distance', 'speed', 'description'
        ])
        writer.writerows(test_cases)
    
    print(f"\n✅ Generated {len(test_cases)} test configurations in {filename}")
    print(f"📊 Configuration:")
    print(f"   - Start: ({start_x}, {start_y})")
    print(f"   - End: ({end_x}, {end_y})")
    print(f"   - Obstacles: {obstacles}")
    print(f"   - Barrier distances: {barrier_distances}")
    print(f"   - Speed: {speed}")
    
    input("\nPress Enter to continue...")

if __name__ == "__main__":
    create_sample_test_file()
    create_comprehensive_test_file()
    create_advanced_test_file()