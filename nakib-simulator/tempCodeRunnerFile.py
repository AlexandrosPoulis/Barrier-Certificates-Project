import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import math

class MovingCar:
    def __init__(self, start_point, end_point, speed=0.02):
        self.start_point = np.array(start_point, dtype=float)
        self.end_point = np.array(end_point, dtype=float)
        self.speed = speed  # Fraction of total distance per frame
        
        # Calculate total distance and direction
        self.total_distance = np.linalg.norm(self.end_point - self.start_point)
        self.direction = (self.end_point - self.start_point) / self.total_distance
        
        # Initialize car position
        self.current_position = self.start_point.copy()
        self.progress = 0.0
        
        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()
        
        # Car dimensions
        self.car_length = 0.8
        self.car_width = 0.4
        
    def setup_plot(self):
        """Setup the plot with proper limits and labels"""
        # Calculate plot limits with some padding
        min_x = min(self.start_point[0], self.end_point[0]) - 1
        max_x = max(self.start_point[0], self.end_point[0]) + 1
        min_y = min(self.start_point[1], self.end_point[1]) - 1
        max_y = max(self.start_point[1], self.end_point[1]) + 1
        
        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('Moving Car Animation')
        
        # Plot the route
        self.ax.plot([self.start_point[0], self.end_point[0]], 
                    [self.start_point[1], self.end_point[1]], 
                    'k--', alpha=0.5, label='Route')
        
        # Mark start and end points
        self.ax.plot(*self.start_point, 'go', markersize=10, label='Start')
        self.ax.plot(*self.end_point, 'ro', markersize=10, label='End')
        
        self.ax.legend()
    
    def calculate_car_angle(self):
        """Calculate the angle of the car based on direction"""
        return math.atan2(self.direction[1], self.direction[0])
    
    def create_car_patches(self, position):
        """Create car body and wheels at given position"""
        angle = self.calculate_car_angle()
        
        # Car body (rectangle)
        car_body = Rectangle((position[0] - self.car_length/2, position[1] - self.car_width/2),
                           self.car_length, self.car_width,
                           angle=np.degrees(angle), rotation_point='center',
                           facecolor='blue', edgecolor='black', alpha=0.8)
        
        # Wheels
        wheel_radius = 0.1
        wheel_positions = [
            (-self.car_length/3, -self.car_width/2 - wheel_radius/2),  # Front left
            (self.car_length/3, -self.car_width/2 - wheel_radius/2),   # Front right
            (-self.car_length/3, self.car_width/2 + wheel_radius/2),   # Rear left
            (self.car_length/3, self.car_width/2 + wheel_radius/2)     # Rear right
        ]
        
        wheels = []
        for wx, wy in wheel_positions:
            # Rotate wheel positions
            rotated_x = wx * math.cos(angle) - wy * math.sin(angle)
            rotated_y = wx * math.sin(angle) + wy * math.cos(angle)
            
            wheel = Circle((position[0] + rotated_x, position[1] + rotated_y),
                         wheel_radius, facecolor='black', alpha=0.7)
            wheels.append(wheel)
        
        return car_body, wheels
    
    def update(self, frame):
        """Update function for animation"""
        # Clear previous car
        for patch in self.ax.patches[:]:
            patch.remove()
        
        # Update position
        self.progress += self.speed
        if self.progress > 1.0:
            self.progress = 0.0  # Reset to start for continuous animation
        
        self.current_position = self.start_point + self.progress * (self.end_point - self.start_point)
        
        # Create new car
        car_body, wheels = self.create_car_patches(self.current_position)
        
        # Add car to plot
        self.ax.add_patch(car_body)
        for wheel in wheels:
            self.ax.add_patch(wheel)
        
        # Update title with current position
        self.ax.set_title(f'Moving Car Animation - Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})')
        
        return [car_body] + wheels
    
    def animate(self):
        """Start the animation"""
        # Create initial car
        car_body, wheels = self.create_car_patches(self.start_point)
        self.ax.add_patch(car_body)
        for wheel in wheels:
            self.ax.add_patch(wheel)
        
        # Start animation
        self.animation = FuncAnimation(self.fig, self.update, frames=None,
                                     interval=50, blit=False, repeat=True)
        
        plt.show()

def get_user_coordinates():
    """Get start and end coordinates from user"""
    print("Enter coordinates for the start and end points:")
    
    try:
        start_x = float(input("Start point X coordinate: "))
        start_y = float(input("Start point Y coordinate: "))
        end_x = float(input("End point X coordinate: "))
        end_y = float(input("End point Y coordinate: "))
        
        return (start_x, start_y), (end_x, end_y)
    except ValueError:
        print("Please enter valid numbers!")
        return get_user_coordinates()

def main():
    """Main function to run the animation"""
    print("Moving Car Animation")
    print("====================")
    
    # Get coordinates from user
    start_point, end_point = get_user_coordinates()
    
    # Create and run animation
    car_animation = MovingCar(start_point, end_point, speed=0.02)
    car_animation.animate()

if __name__ == "__main__":
    main()