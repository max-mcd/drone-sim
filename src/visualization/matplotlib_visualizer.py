"""2D visualization of the drone simulation using matplotlib

TODO Visualization Improvements:
1. Trail Customization
   - Add max trail length to prevent memory issues
   - Add trail color based on drone speed
   - Add trail fade-out effect
   - Add option to toggle trails

2. Info Bubbles
   - Add distance to destination
   - Add battery level
   - Add collision warning indicators
   - Make bubble position adjustable to prevent overlap

3. Velocity Vectors
   - Add arrow showing drone direction
   - Scale arrow length with speed
   - Add color gradient based on speed

4. Performance
   - Optimize trail storage
   - Add frame skip option for faster simulation
   - Add visualization buffer

5. Interaction
   - Add pause/resume controls
   - Add speed controls
   - Add zoom/pan controls
   - Add drone selection for detailed view
"""

import logging
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from ..models.simulation_state import SimulationState

# Configure logging after imports but before any matplotlib usage
logging.getLogger('matplotlib').setLevel(logging.ERROR)
logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.ERROR)

# Suppress matplotlib warnings and debug messages
plt.set_loglevel('warning')  # This is the correct way to set matplotlib's log level

# Initialize logger for this module
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)  # Set to DEBUG to see all position updates


class MatplotlibVisualizer:
    """2D visualization of the drone simulation using matplotlib"""
    
    def __init__(self, dimensions: Tuple[float, float, float], real_time: bool = True):
        """Initialize the visualizer with given dimensions and mode
        
        Args:
            dimensions: (width, length, height) of simulation space
            real_time: If True, updates display in real-time. If False, runs faster
        """
        plt.style.use('dark_background')  # Better visibility
        self.real_time = real_time
        self.dimensions = dimensions
        
        # Create figure and axes with wider figure to accommodate legend
        self.fig = plt.figure(figsize=(14, 8))  # Increased from (12, 8)
        self.ax = self.fig.add_subplot(111)
        
        # Add more right padding for legend
        plt.subplots_adjust(right=0.85)  # Adjust right margin for legend
        
        # Setup paths and storage
        self.output_dir = Path.cwd() / "output"
        self.output_dir.mkdir(exist_ok=True)
        self.drone_trails = {}
        self.max_trail_length = 50  # Limit trail memory usage
        self.current_state = None  # Add this line
        
        # Create legend elements
        self.legend_elements = [
            plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
                      markersize=10, label='Active Drones'),
            plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='red',
                      markersize=10, label='Collided Drones'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
                      markersize=10, label='Successful Drones'),
            plt.Rectangle((0,0), 1, 1, fc='gray', alpha=0.5, label='Buildings'),
            plt.Line2D([0], [0], color='lightblue', linestyle='--',
                      label='Drone Trail'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
                      markersize=10, label='Start Points'),
            plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='yellow',
                      markersize=10, label='Destinations')
        ]
        
        self.setup_plot()
        
    def setup_plot(self) -> None:
        """Initialize plot styling and axes"""
        # Set bounds with padding
        padding = 50  # meters
        self.ax.set_xlim(-padding, self.dimensions[0] + padding)
        self.ax.set_ylim(-padding, self.dimensions[1] + padding)
        
        # Style
        self.ax.set_title("Drone Flight Simulation", pad=20, fontsize=14)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        
        # Add legend
        self.ax.legend(
            handles=self.legend_elements,
            loc='upper right',
            bbox_to_anchor=(1.15, 1),
            fontsize=10
        )
        
        if self.real_time:
            plt.ion()
        
    def on_state_update(self, state: SimulationState) -> None:
        """Update visualization with new simulation state"""
        self.current_state = state
        
        # Clear previous plot elements
        self.ax.clear()
        
        # Log all drone positions and states
        logger.info(f"""
            Visualization Update at Time: {state.time:.1f}s
            ----------------------------------------""")
        for drone in state.drones:
            pos = drone['position']
            vel = drone['velocity']
            speed = np.linalg.norm(vel)
            dist_to_dest = np.linalg.norm(np.array(pos) - np.array(drone['destination']))
            
            logger.info(f"""
            Drone {drone['id']}:
                Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})
                Status: {drone['status']}
                Speed: {speed:.2f} m/s
                Distance to destination: {dist_to_dest:.2f}m
            """)
        
        # Plot buildings first (background)
        self._plot_buildings(state.buildings)
        
        # Plot drones and their trails
        for drone in state.drones:
            self._update_trails(drone)
            self._plot_drone(drone)
        
        # Plot collision points
        self._plot_collisions(state.building_collisions)
        
        # Update plot settings
        self.setup_plot()
        
        # Draw and pause if in real-time mode
        if self.real_time:
            plt.draw()
            plt.pause(0.01)  # Small pause to allow for visualization
        else:
            plt.draw()
            plt.pause(0.001)  # Faster updates in fast mode

    def _update_trails(self, drone: dict) -> None:
        """Update and manage trail for a single drone"""
        pos = drone['position']
        drone_id = drone['id']
        
        if drone_id not in self.drone_trails:
            self.drone_trails[drone_id] = []
            
        self.drone_trails[drone_id].append(pos)
        
        # Limit trail length
        if len(self.drone_trails[drone_id]) > self.max_trail_length:
            self.drone_trails[drone_id].pop(0)
            
        # Plot trail if exists
        if len(self.drone_trails[drone_id]) > 1:
            trail = np.array(self.drone_trails[drone_id])
            alpha_values = np.linspace(0.1, 0.5, len(trail))  # Fade effect
            
            for i in range(len(trail) - 1):
                self.ax.plot(
                    trail[i:i+2, 0],
                    trail[i:i+2, 1],
                    color='lightblue',
                    alpha=alpha_values[i],
                    linestyle='--'
                )

    def _plot_drone(self, drone: dict) -> None:
        """Plot a single drone with its info bubble and waypoints"""
        pos = drone['position']
        vel = drone['velocity']
        speed = np.linalg.norm(vel)
        
        # Detailed position logging for Drone 2
        if drone['id'] == 2:
            logger.info(f"""
                Visualization update for Drone 2:
                Time: {self.current_state.time:.1f}s
                Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})
                Status: {drone['status']}
                Speed: {speed:.2f} m/s
                Distance to destination: {np.linalg.norm(np.array(pos) - np.array(drone['destination'])):.2f}m
            """)
        
        # Plot start point (blue circle)
        self.ax.scatter(
            drone['start_pos'][0], drone['start_pos'][1],
            c='blue', marker='o', s=100, zorder=2
        )
        
        # Plot destination (yellow star)
        self.ax.scatter(
            drone['destination'][0], drone['destination'][1],
            c='yellow', marker='*', s=100, zorder=2
        )
        
        # Determine drone marker and color based on status
        if drone['status'] == 'collided':
            marker = '*'  # Star for collided drones
            color = 'red'
            bubble_color = 'red'
            bubble_text_color = 'white'
        elif drone['status'] == 'successful':
            marker = 'o'  # Circle for successful drones
            color = 'green'
            bubble_color = 'yellow'
            bubble_text_color = 'black'
        else:
            marker = '*'  # Star for active drones
            color = 'green'
            bubble_color = 'yellow'
            bubble_text_color = 'black'
        
        # Plot current drone position
        self.ax.scatter(
            pos[0], pos[1],
            c=color, marker=marker, s=100, zorder=3
        )
        
        # Add collision detection radius visualization
        safety_circle = plt.Circle(
            (pos[0], pos[1]),
            radius=5.0,  # Half of COLLISION_THRESHOLD
            color='red',
            fill=False,
            alpha=0.2,
            linestyle=':',
            zorder=2
        )
        self.ax.add_patch(safety_circle)
        
        # Add velocity vector with speed indicator
        if speed > 0:
            # Calculate time to potential collision
            for other_drone in self.current_state.drones:
                if other_drone['id'] != drone['id']:
                    dist = np.linalg.norm(np.array(pos) - np.array(other_drone['position']))
                    if dist < 100:  # Check nearby drones
                        time_to_closest = self._calculate_time_to_closest_approach(
                            pos, vel, 
                            other_drone['position'], 
                            other_drone['velocity']
                        )
                        if time_to_closest is not None:
                            self.ax.text(
                                pos[0], pos[1] + 20,
                                f'Time to closest: {time_to_closest:.1f}s',
                                color='yellow'
                            )
        
        # Add info bubble with color based on status
        info_text = (
            f"ID: {drone['id']}\n"
            f"Speed: {speed:.1f} m/s\n"
            f"Alt: {pos[2]:.1f}m"
        )
        self.ax.annotate(
            info_text,
            xy=(pos[0], pos[1]),
            xytext=(10, 10),
            textcoords='offset points',
            bbox=dict(
                boxstyle='round,pad=0.5',
                fc=bubble_color,
                alpha=0.7
            ),
            fontsize=8,
            color=bubble_text_color
        )

    def _plot_buildings(self, buildings: List[dict]) -> None:
        """Plot buildings as semi-transparent rectangles
        
        Args:
            buildings: List of building dictionaries with position and dimensions
        """
        for building in buildings:
            pos = building['position']
            dims = building['dimensions']
            
            # Create rectangle patch
            rect = plt.Rectangle(
                (pos[0] - dims[0]/2, pos[1] - dims[1]/2),  # Bottom left corner
                dims[0], dims[1],  # Width and length
                color='gray',
                alpha=0.5,
                zorder=1  # Ensure buildings are behind drones
            )
            
            self.ax.add_patch(rect)

    def _plot_collisions(self, collisions: List[Tuple]) -> None:
        """Plot collision points with warning indicators"""
        if not collisions:
            return
            
        # Plot each collision point
        for collision in collisions:
            x, y = collision[3], collision[4]  # Get collision coordinates
            
            # Plot collision marker
            self.ax.scatter(
                x, y,
                c='yellow',
                marker='x',
                s=100,
                zorder=3  # Ensure visible above other elements
            )
            
            # Add warning circle
            warning_circle = plt.Circle(
                (x, y),
                radius=10,  # 10m radius
                color='yellow',
                fill=False,
                alpha=0.3,
                linestyle=':',
                zorder=2
            )
            self.ax.add_patch(warning_circle)

    def save_plot(self, filename: str = "final_state.png") -> None:
        """Save current plot state to file"""
        save_path = self.output_dir / filename
        self.fig.savefig(save_path)
        print(f"\nFinal state saved to: {save_path}")

    def _calculate_time_to_closest_approach(
        self, 
        pos1: List[float], 
        vel1: List[float], 
        pos2: List[float], 
        vel2: List[float]
    ) -> float:
        """Calculate time until closest approach between two drones
        
        Args:
            pos1: Position vector of first drone [x,y,z]
            vel1: Velocity vector of first drone [vx,vy,vz]
            pos2: Position vector of second drone [x,y,z]
            vel2: Velocity vector of second drone [vx,vy,vz]
            
        Returns:
            Time (in seconds) until closest approach, or None if drones are moving apart
        """
        pos1 = np.array(pos1)
        pos2 = np.array(pos2)
        vel1 = np.array(vel1)
        vel2 = np.array(vel2)
        
        # Calculate relative position and velocity
        rel_pos = pos1 - pos2
        rel_vel = vel1 - vel2
        
        # If relative speed is zero, return None
        if np.allclose(rel_vel, 0):
            return None
            
        # Calculate time to closest approach
        # This is when relative position dot relative velocity = 0
        t = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        
        # Only return positive times (future collisions)
        return t if t > 0 else None 