import logging
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from ..models.building import Building
from ..models.simulation_state import SimulationState

# Configure matplotlib logging
logging.getLogger('matplotlib').setLevel(logging.ERROR)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

class MatplotlibVisualizer:
    """2D visualization of the drone simulation using matplotlib
    
    Visualization features:
    1. Real-time drone position and status tracking
    2. Building layout visualization
    3. Collision detection visualization
    4. Flight path and waypoint tracking
    5. Performance metrics display
    6. Dynamic legend and status indicators
    """
    
    def __init__(self, dimensions: Tuple[float, float, float], real_time: bool = True):
        """Initialize the visualizer with given dimensions and mode
        
        Args:
            dimensions: (width, length, height) of simulation space
            real_time: If True, updates display in real-time. If False, runs faster
        """
        
        # Use Agg backend if not in main thread
        import threading
        if threading.current_thread() is not threading.main_thread():
            import matplotlib
            matplotlib.use('Agg')
        
        # Store basic parameters
        self.dimensions = dimensions
        self.real_time = real_time
        plt.style.use('dark_background')
        
        # Initialize state tracking
        self.received_first_state = False
        self.current_state = None
        
        # Create figure and axes first
        self.fig = plt.figure(figsize=(16, 9))
        self.ax = self.fig.add_subplot(111)
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.85, top=0.95)
        
        # Track visual elements
        self.drones_scatter = {}      # {drone_id: scatter_object}
        self.drone_trails = {}        # {drone_id: list_of_line_objects}
        self.collision_artists = []    # List of collision markers
        self.info_bubbles = {}        # {drone_id: annotation_object}
        self.waypoint_markers = {}    # {drone_id: {start, end, current}}
        self.safety_circles = {}      # {drone_id: circle_object}
        self.time_label = None        # Will be created in _update_time_label
        self.building_patches = []     # List of building patches
        
        # Setup static plot elements
        self._static_plot_setup()
        
        # Setup storage
        self.output_dir = Path.cwd() / "output"
        self.output_dir.mkdir(exist_ok=True)
        self.drone_trails = {}
        self.max_trail_length = 50
        
        # Create legend elements
        self.legend_elements = [
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='green',
                       markeredgecolor='white', markersize=10, label='Active Drones'),
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='red',
                       markeredgecolor='white', markersize=10, label='Collided Drones'),
            plt.Line2D([0], [0], marker='o', color='none', markerfacecolor='green',
                       markeredgecolor='white', markersize=10, label='Successful Drones'),
            plt.Rectangle((0,0), 1, 1, fc='gray', alpha=0.5, label='Buildings'),
            plt.Line2D([0], [0], linestyle='--', color='lightblue', label='Drone Trail'),
            plt.Line2D([0], [0], marker='o', color='none', markerfacecolor='blue',
                       markeredgecolor='white', markersize=10, label='Start Points'),
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='yellow',
                       markeredgecolor='white', markersize=10, label='Destinations'),
            plt.Line2D([0], [0], marker='x', color='none', markerfacecolor='none',
                       markeredgecolor='orange', markersize=10, markeredgewidth=2,
                       label='Collision Point')
        ]
        
        # Default buffer settings - will be adjusted when first state arrives
        self.max_buffer_size = 100 if real_time else 1000
        self.frame_skip = 2 if real_time else 5
        self.animation_interval = 50  # Faster updates for smoother animation
        self.is_drawing = False  # Lock to prevent recursive drawing
        
        self.state_buffer = []
        self.last_state = None
        
        # Add completion callback
        self.on_simulation_complete = None
        
        # Add completion flag
        self.completion_pending = False
        self.completion_time = None
        
        # Set up the animation after everything else is initialized
        plt.ion() if real_time else plt.ioff()
        plt.show(block=False)
        
        # Start animation timer
        self._setup_animation()
        
        # Create initial time label
        self._update_time_label()
        
        logger.debug("Visualizer initialization complete")
        
    def setup_plot(self) -> None:
        """Initialize plot styling and axes"""
        logger.debug("Setting up plot...")
        self.ax.set_xlim(-50, self.dimensions[0] + 50)
        self.ax.set_ylim(-50, self.dimensions[1] + 50)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_title("Drone Flight Simulation")
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        
        # Move legend to right side of plot
        self.ax.legend(
            handles=self.legend_elements, 
            loc='center left',           # Align left edge of legend
            bbox_to_anchor=(1.02, 0.5),  # Place just outside right edge of plot
            fontsize=9
        )
        logger.debug("Plot setup complete")
        
    def _animation_update(self, frame):
        """Update dynamic elements without clearing the plot"""
        if self.is_drawing or not self.current_state:
            return
            
        try:
            self.is_drawing = True
            
            # Clear previous frame's dynamic elements
            self._clear_dynamic_elements()
            
            # Update all visual elements
            self._update_all_drones()
            self._update_collision_markers()
            self._update_time_label()
            
            # Use a single draw call
            self.fig.canvas.draw_idle()
        except Exception as e:
            logger.error(f"Error updating animation: {e}")
        finally:
            self.is_drawing = False

    def _clear_dynamic_elements(self):
        """Clear all dynamic elements from previous frame"""
        # Clear drones and their associated elements
        for drone_id in list(self.drones_scatter.keys()):
            self.drones_scatter[drone_id].remove()
            if drone_id in self.info_bubbles:
                self.info_bubbles[drone_id].remove()
            if drone_id in self.waypoint_markers:
                for marker in self.waypoint_markers[drone_id].values():
                    marker.remove()
            # Clear safety circles and other drone-specific elements
            if hasattr(self, 'safety_circles') and drone_id in self.safety_circles:
                self.safety_circles[drone_id].remove()
        
        self.drones_scatter.clear()
        self.info_bubbles.clear()
        self.waypoint_markers.clear()
        if hasattr(self, 'safety_circles'):
            self.safety_circles.clear()
        
        # Clear collision markers
        for artist in self.collision_artists:
            artist.remove()
        self.collision_artists.clear()

    def _update_all_drones(self):
        """Update all drone visual elements"""
        for drone in self.current_state.drones:
            self._plot_drone(drone)  # Use the original plotting function that includes all elements

    def on_state_update(self, state: SimulationState) -> None:
        """Handle new simulation state update"""
        try:
            # Store current state
            self.current_state = state
            
            # Initialize buildings on first state
            if not self.received_first_state:
                self.received_first_state = True
                self._initialize_building_patches(state.buildings)
                
                # Add building patches to plot
                for patch in self.building_patches:
                    self.ax.add_patch(patch)
            
            # Update visualization immediately in non-real-time mode
            if not self.real_time:
                self._animation_update(None)
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            
            # Check if simulation is complete
            if all(drone['status'] in ['successful', 'collided'] for drone in state.drones):
                self.save_plot()
            
        except Exception as e:
            logger.error(f"State update failed: {e}")

    def _initialize_building_patches(self, buildings: List[Building]) -> None:
        """Create building patches once during initialization"""
        logger.debug(f"Initializing {len(buildings)} building patches")
        self.building_patches = []
        
        for i, building in enumerate(buildings):
            try:
                rect = plt.Rectangle(
                    (building.x - building.width/2,  # Direct property access
                     building.y - building.length/2),
                    building.width,
                    building.length,
                    color='gray',
                    alpha=0.5,
                    zorder=1
                )
                self.building_patches.append(rect)
                if i < 5:  # Log first few buildings for verification
                    logger.debug(f"Building {i}: pos=({building.x}, {building.y}), "
                               f"size={building.width}x{building.length}")
            except Exception as e:
                logger.error(f"Failed to create building {i}: {e}")
        
        logger.debug(f"Created {len(self.building_patches)} building patches")
        
        # Force a redraw to show buildings immediately
        self.fig.canvas.draw()

    def _plot_collisions(self, collisions: List[Tuple]) -> None:
        """Plot collision points with warning indicators"""
        if not collisions:
            return
            
        # Plot each collision point that has occurred
        for collision in collisions:
            # Building collisions have 6 elements (drone_id, building_id, time, x, y, z)
            # Drone collisions have 3 elements (drone1_id, drone2_id, time)
            if len(collision) == 6:  # Building collision
                x, y = collision[3], collision[4]  # Get collision coordinates
                color = 'orange'  # Different color for building collisions
                collision_time = collision[2]
            else:  # Drone collision
                drone1 = next(d for d in self.current_state.drones if d['id'] == collision[0])
                x, y = drone1['position'][0], drone1['position'][1]
                color = 'red'
                collision_time = collision[2]
            
            # Plot collision marker
            self.ax.scatter(
                x, y,
                c=color,
                marker='x',
                s=100,
                zorder=3  # Ensure visible above other elements
            )
            
            # Add warning circle
            warning_circle = plt.Circle(
                (x, y),
                radius=10,  # 10m radius
                color=color,
                fill=False,
                alpha=0.3,
                linestyle=':',
                zorder=2
            )
            self.ax.add_patch(warning_circle)
            
            # Add collision time label
            self.ax.text(
                x, y - 15,
                f'Collision at {collision_time:.1f}s',
                color=color,
                fontsize=8,
                ha='center'
            )

    def stop_animation(self):
        """Properly stop the animation"""
        # Don't try to stop the animation - just let it run
        pass

    def save_plot(self) -> None:
        """Save the current plot state to a file"""
        try:
            # Make sure we have a valid figure before saving
            if not plt.fignum_exists(self.fig.number):
                logger.debug("Skipping plot save: Figure already closed")
                return
                
            # Force a redraw of the figure
            self.fig.canvas.draw()
            
            # Save to file
            save_path = self.output_dir / f"simulation_state_{self.current_state.time:.1f}s.png"
            self.fig.savefig(save_path)
            
            print(f"\nFinal state saved to: {save_path}")
            
        except Exception as e:
            # This is expected when window is closed
            logger.debug(f"Note: Could not save plot - {e}")

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

    def _plot_drone(self, drone: dict) -> None:
        """Plot a single drone with its info bubble and waypoints"""
        pos = drone['position']
        vel = drone['velocity']
        speed = np.linalg.norm(vel)
        drone_id = drone['id']
        waypoints = np.array(drone['flight_path'])
        
        # Initialize containers for this drone's artists
        self.drones_scatter[drone_id] = None  # Will hold the drone marker
        self.waypoint_markers[drone_id] = {}  # Initialize waypoint markers dict
        
        # Plot start point (blue circle with yellow center)
        self.waypoint_markers[drone_id]['start'] = self.ax.scatter(
            waypoints[0][0], waypoints[0][1],
            c='yellow', marker='o', 
            edgecolor='blue',
            s=100, zorder=2
        )
        
        # Plot destination
        self.waypoint_markers[drone_id]['end'] = self.ax.scatter(
            waypoints[-1][0], waypoints[-1][1],  # Use last waypoint
            c='yellow', marker='*',
            s=100, zorder=2
        )
        
        # Plot planned path
        self.ax.plot(
            waypoints[:, 0], waypoints[:, 1],
            'y--', alpha=0.3, zorder=1
        )
        
        # Plot current waypoint
        current = waypoints[drone['current_waypoint_index']]
        self.ax.scatter(
            current[0], current[1],
            c='yellow', marker='o', s=50, zorder=2
        )
        
        # Determine drone marker and color based on status
        if drone['status'] == 'collided':
            marker = '*'
            color = 'red'
        elif drone['status'] == 'successful':
            marker = 'o'  # Change to circle for successful drones
            color = 'green'
        else:
            marker = '*'  # Star for active drones
            color = 'green'
        
        # Plot current drone position
        self.drones_scatter[drone_id] = self.ax.scatter(
            pos[0], pos[1],
            c=color, marker=marker, s=100, zorder=3
        )
        
        # Update drone trail
        if drone_id not in self.drone_trails:
            self.drone_trails[drone_id] = []
        self._update_drone_trail(drone)
        
        # Add info bubble
        info_text = (
            f"ID: {drone['id']}\n"
            f"Speed: {speed:.1f} m/s\n"
            f"Alt: {pos[2]:.1f}m"
        )
        
        bubble_color = {
            'collided': 'darkred',
            'successful': 'darkgreen',
            'active': 'darkblue'
        }[drone['status']]
        
        self.info_bubbles[drone['id']] = self.ax.annotate(
            info_text,
            xy=(pos[0], pos[1]),
            xytext=(10, 10),
            textcoords='offset points',
            bbox=dict(boxstyle='round,pad=0.5', fc=bubble_color, alpha=0.7),
            fontsize=8,
            color='white'
        )

    def _static_plot_setup(self):
        """
        Set up static aspects of the plot that don't need to be 
        re-done every frame (axes labels, grid, legend, etc.).
        """
        self.ax.set_xlim(-50, self.dimensions[0] + 50)
        self.ax.set_ylim(-50, self.dimensions[1] + 50)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_title("Drone Flight Simulation")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        
        # Complete legend elements
        self.legend_elements = [
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='green',
                    markeredgecolor='white', markersize=10, label='Active Drones'),
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='red',
                    markeredgecolor='white', markersize=10, label='Collided Drones'),
            plt.Line2D([0], [0], marker='o', color='none', markerfacecolor='green',
                    markeredgecolor='white', markersize=10, label='Successful Drones'),
            plt.Rectangle((0,0), 1, 1, fc='gray', alpha=0.5, label='Buildings'),
            plt.Line2D([0], [0], linestyle='--', color='lightblue', label='Drone Trail'),
            plt.Line2D([0], [0], marker='o', color='none', markerfacecolor='blue',
                    markeredgecolor='white', markersize=10, label='Start Points'),
            plt.Line2D([0], [0], marker='*', color='none', markerfacecolor='yellow',
                    markeredgecolor='white', markersize=10, label='Destinations'),
            plt.Line2D([0], [0], marker='x', color='none', markerfacecolor='none',
                    markeredgecolor='orange', markersize=10, markeredgewidth=2,
                    label='Collision Point')
        ]
        
        self.ax.legend(
            handles=self.legend_elements,
            loc='center left',
            bbox_to_anchor=(1.02, 0.5),
            fontsize=9
        )

    def _update_time_label(self):
        """Update simulation time display"""
        # Use current state time if available, otherwise show 0.0
        time = self.current_state.time if self.current_state else 0.0
        time_text = f"Time: {time:.1f}s"
        
        # Remove or update existing text object
        if hasattr(self, 'time_label') and self.time_label is not None:
            self.time_label.remove()
        
        self.time_label = self.fig.text(
            0.02, 0.02,
            time_text,
            fontsize=10,
            color='white',
            bbox=dict(facecolor='black', alpha=0.7, edgecolor='none')
        )

    def _setup_drone_visuals(self, drone: dict) -> None:
        """Create initial visual elements for a drone"""
        drone_id = drone['id']
        pos = drone['position']
        waypoints = np.array(drone['flight_path'])
        
        # Create scatter for drone position
        self.drones_scatter[drone_id] = self.ax.scatter(
            pos[0], pos[1],
            c='green', marker='*', s=100, zorder=3
        )
        
        # Create waypoint markers
        self.waypoint_markers[drone_id] = {
            'start': self.ax.scatter(
                waypoints[0][0], waypoints[0][1],
                c='blue', marker='o', s=100, zorder=2
            ),
            'end': self.ax.scatter(
                waypoints[-1][0], waypoints[-1][1],
                c='yellow', marker='*', s=100, zorder=2
            ),
            'current': self.ax.scatter(
                waypoints[0][0], waypoints[0][1],
                c='yellow', marker='o', s=50, zorder=2
            )
        }
        
        # Initialize empty trail
        self.drone_trails[drone_id] = []

    def _get_drone_color(self, status: str) -> str:
        return {
            'collided': 'red',
            'successful': 'green',
            'active': 'green'
        }[status]

    def _update_drone_trail(self, drone: dict) -> None:
        drone_id = drone['id']
        pos = drone['position']
        
        # Add new trail segment
        if len(self.drone_trails[drone_id]) >= self.max_trail_length:
            old_line = self.drone_trails[drone_id].pop(0)
            old_line.remove()
            
        line = self.ax.plot(
            [pos[0]], [pos[1]],
            color='lightblue',
            alpha=0.5,
            linestyle='--'
        )[0]
        self.drone_trails[drone_id].append(line)

    def _update_collision_markers(self) -> None:
        # Add new collision markers
        current_collisions = [
            c for c in (self.current_state.drone_collisions + 
                       self.current_state.building_collisions)
            if c[2] <= self.current_state.time
        ]
        
        for collision in current_collisions:
            # Building collisions have 6 elements (drone_id, building_id, time, x, y, z)
            # Drone collisions have 3 elements (drone1_id, drone2_id, time)
            if len(collision) == 6:  # Building collision
                x, y = collision[3], collision[4]  # Get collision coordinates
                color = 'orange'  # Different color for building collisions
                collision_time = collision[2]
            else:  # Drone collision
                drone1 = next(d for d in self.current_state.drones if d['id'] == collision[0])
                x, y = drone1['position'][0], drone1['position'][1]
                color = 'red'
                collision_time = collision[2]
            
            # Plot collision marker
            marker = self.ax.scatter(
                x, y,
                c=color,
                marker='x',
                s=100,
                zorder=3
            )
            
            # Add warning circle
            warning_circle = plt.Circle(
                (x, y),
                radius=10,
                color=color,
                fill=False,
                alpha=0.3,
                linestyle=':',
                zorder=2
            )
            self.ax.add_patch(warning_circle)
            
            # Add collision time label
            label = self.ax.text(
                x, y - 15,
                f'Collision at {collision_time:.1f}s',
                color=color,
                fontsize=8,
                ha='center'
            )
            
            # Store all artists for later removal
            self.collision_artists.extend([marker, warning_circle, label])

    def _setup_animation(self):
        """Set up the animation timer"""
        self.ani = FuncAnimation(
            self.fig,
            self._animation_update,
            interval=self.animation_interval,
            blit=False,
            cache_frame_data=False
        )