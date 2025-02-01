import logging
import warnings
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from ..models.building import Building
from ..models.drone import DroneStatus
from ..models.simulation_state import SimulationState

# Configure matplotlib logging
logging.getLogger('matplotlib').setLevel(logging.ERROR)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

class MatplotlibVisualizer:
    """2D visualization of the drone simulation using matplotlib.

    Provides real-time visualization of drone flights, building layouts, and simulation metrics.

    Visualization features:
        - Real-time drone position and status tracking: Shows current location and state of each drone
        - Building layout visualization: Displays 2D projections of buildings in the simulation space
        - Collision detection visualization: Highlights collisions between drones and with buildings
        - Flight path and waypoint tracking: Shows planned routes and progress of each drone
        - Performance metrics display: Real-time stats like simulation time and drone counts
        - Dynamic legend and status indicators: Color-coded markers showing drone states

    The visualizer updates in real-time when real_time=True, or runs faster without display updates
    when real_time=False.
    """
    
    def __init__(self, dimensions: Tuple[float, float, float], real_time: bool = True):
        """Initialize the visualizer with given dimensions and mode.
        
        The visualizer provides real-time visualization of:
        - Drone positions and status (using color-coded markers)
        - Building layouts (as 2D projections)
        - Flight paths and waypoints
        - Collision detection
        - Performance metrics
        
        Args:
            dimensions: (width, length, height) tuple defining simulation space dimensions in meters
            real_time: If True, updates display in real-time. If False, runs without display updates
                      for faster simulation speed
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
        self.drone_colors = {}        # {drone_id: color}
        self.drones_scatter = {}      # {drone_id: scatter_object}
        self.drone_trails = {}        # {drone_id: list_of_line_objects}
        self.collision_artists = []    # List of collision markers
        self.info_bubbles = {}        # {drone_id: annotation_object}
        self.waypoint_markers = {}    # {drone_id: {start, end, current}}
        self.time_label = None        # Will be created in _update_time_label
        self.building_patches = []     # List of building patches
        
        # Initialize marker mapping
        self.drone_markers = {
            DroneStatus.ACTIVE: '*',           # Star for active drones
            DroneStatus.SUCCESSFUL.value: 'o',  # Need to use .value since status comes from dict
            DroneStatus.COLLIDED: '*',         # Star for collided drones
            DroneStatus.BATTERY_DEPLETED: 's'  # Square for battery depleted
        }
        
        # Initialize color scheme
        self.status_colors = {
            DroneStatus.ACTIVE: 'green',        # Green for active drones
            DroneStatus.SUCCESSFUL.value: 'green', # Need to use .value to match status from dict
            DroneStatus.COLLIDED: 'red',        # Red for collisions
            DroneStatus.BATTERY_DEPLETED: 'orange'
        }
        
        # Initialize trail properties
        self.max_trail_length = 50  # Maximum number of trail points to keep
        self.trail_alpha = 0.3      # Transparency of trail lines
        
        # Setup static plot elements
        self._static_plot_setup()
        
        # Setup storage
        self.output_dir = Path.cwd() / "output"
        self.output_dir.mkdir(exist_ok=True)
        
        self.animation_interval = 50  # Faster updates for smoother animation
        self.is_drawing = False  # Lock to prevent recursive drawing
        
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
            if drone_id in self.drones_scatter and self.drones_scatter[drone_id] is not None:
                self.drones_scatter[drone_id].remove()
            
            if drone_id in self.info_bubbles:
                self.info_bubbles[drone_id].remove()
            
            if drone_id in self.waypoint_markers and isinstance(self.waypoint_markers[drone_id], dict):
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
            if artist is not None:
                try:
                    artist.remove()
                except ValueError:
                    pass  # Artist was already removed
        self.collision_artists.clear()

    def _update_all_drones(self):
        """Update drone positions and trails"""
        if not self.current_state:
            return
        
        for drone in self.current_state.drones:
            waypoints = drone['flight_path']
            
            # Plot waypoints if not already plotted
            if drone['id'] not in self.waypoint_markers:
                # Plot start point
                self.ax.scatter(
                    waypoints[0][0], waypoints[0][1],
                    color='blue', marker='o', s=100, zorder=2
                )
                
                # Plot intermediate waypoints
                if len(waypoints) > 2:
                    self.ax.scatter(
                        [wp[0] for wp in waypoints[1:-1]],
                        [wp[1] for wp in waypoints[1:-1]],
                        color='yellow', marker='o', s=50, zorder=2
                    )
                
                # Plot destination
                self.ax.scatter(
                    waypoints[-1][0], waypoints[-1][1],
                    color='yellow', marker='*', s=100, zorder=2
                )
                
                # Store that we've plotted waypoints
                self.waypoint_markers[drone['id']] = True
            
            # Get marker and status-based color
            status = DroneStatus(drone['status'])  # Convert string to enum
            marker = self.drone_markers.get(status, self.drone_markers[DroneStatus.ACTIVE])
            status_color = self.status_colors.get(status, self.status_colors[DroneStatus.ACTIVE])
            
            # Get bubble color based on status
            bubble_color = {
                DroneStatus.ACTIVE.value: 'blue',
                DroneStatus.SUCCESSFUL.value: 'green',
                DroneStatus.COLLIDED.value: 'red',
                DroneStatus.BATTERY_DEPLETED.value: 'orange'
            }.get(drone['status'], 'blue')
            
            # Plot drone position with status color
            self.drones_scatter[drone['id']] = self.ax.scatter(
                drone['position'][0], drone['position'][1],
                color=status_color,
                marker=marker,
                s=100,
                zorder=3
            )
            
            # Add info bubble with status-specific color
            info_text = (
                f"ID: {drone['id']}\n"
                f"Speed: {np.linalg.norm(drone['velocity']):.1f} m/s\n"
                f"Alt: {drone['position'][2]:.1f}m"
            )
            
            self.info_bubbles[drone['id']] = self.ax.annotate(
                info_text,
                xy=(drone['position'][0], drone['position'][1]),
                xytext=(10, 10),
                textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.5', fc=bubble_color, alpha=0.7),
                fontsize=8,
                color='white'
            )
            
            # Update drone trail with drone-specific color
            if drone['id'] not in self.drone_trails:
                self.drone_trails[drone['id']] = []
            self._update_drone_trail(drone)

    def on_state_update(self, state: SimulationState) -> None:
        """Handle new simulation state update"""
        try:
            # Check if figure is still valid
            if self.fig is None or not plt.fignum_exists(self.fig.number):
                logger.debug("Skipping state update - figure was closed")
                return
            
            # Store current state
            self.current_state = state
            
            # Initialize buildings on first state
            if not self.received_first_state:
                self.received_first_state = True
                self._create_building_patches(state.buildings)
                
                # Add building patches to plot
                for patch in self.building_patches:
                    self.ax.add_patch(patch)
            
            # Update visualization immediately in non-real-time mode
            if not self.real_time:
                self._animation_update(None)
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            
            # Check if simulation is complete
            if all(drone['status'] in [DroneStatus.SUCCESSFUL.value, DroneStatus.COLLIDED.value] for drone in state.drones):
                self.save_plot()
            
        except Exception as e:
            logger.error(f"State update failed: {e}")

    def _create_building_patches(self, buildings):
        """Create rectangle patches for buildings"""
        self.building_patches = []
        
        for i, building in enumerate(buildings):
            try:
                # Create rectangle patch for building footprint
                rect = plt.Rectangle(
                    (building['x'] - building['width']/2, building['y'] - building['length']/2),
                    building['width'],
                    building['length'],
                    color='gray',
                    alpha=0.5,
                    zorder=1
                )
                self.ax.add_patch(rect)
                self.building_patches.append(rect)
            except Exception as e:
                logger.error(f"Failed to create building {i}: {e}")
        
        logger.debug(f"Created {len(self.building_patches)} building patches")
        
        # Force a redraw to show buildings immediately
        self.fig.canvas.draw()

    def _update_collision_markers(self) -> None:
        """Update collision markers for all collisions that have occurred"""
        # Add new collision markers
        current_collisions = [
            c for c in self.current_state.collisions
            if c['timestamp'] <= self.current_state.time
        ]
        
        for collision in current_collisions:
            if collision['collision_type'] == 'building':
                x, y = collision['position'][0], collision['position'][1]  # Get collision coordinates
                color = 'orange'  # Different color for building collisions
                collision_time = collision['timestamp']
            else:  # Drone collision
                drone1 = next(d for d in self.current_state.drones if d['id'] == collision['drone_id'])
                x, y = drone1['position'][0], drone1['position'][1]
                color = 'red'
                collision_time = collision['timestamp']
            
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

    def save_plot(self) -> None:
        """Save the current plot state to a validated file path"""
        if not self.current_state:
            logger.warning("No simulation state available for saving")
            return
            
        # Check if figure is still valid
        if self.fig is None or not plt.fignum_exists(self.fig.number) or not hasattr(self.fig.canvas, 'get_renderer'):
            logger.debug("Cannot save plot - figure was closed or not properly initialized")
            return
            
        try:
            # Validate output directory first
            self.output_dir.mkdir(exist_ok=True, parents=True)
            save_path = self.output_dir / f"sim_state_{self.current_state.time:.1f}s.png"
            export_path = str(save_path.resolve())

            # Ensure figure is properly rendered before saving
            self.fig.canvas.draw()
            
            # Perform actual drawing/saving in one atomic operation
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                self.fig.savefig(export_path, bbox_inches='tight')

            logger.info(f"Successfully saved simulation state to:\n{export_path}")
            
        except RuntimeError as e:
            if "closed figure" in str(e):
                logger.debug("Save attempted on closed figure")
            else:
                logger.error(f"Matplotlib error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error saving plot: {e}")

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
            Line2D([0], [0], marker='*', color='w', markerfacecolor='green',
                    label='Active Drones', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
                    label='Successful Drones', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='red',
                    label='Collided Drones', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='o', color='blue',
                    label='Start Points', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='*', color='yellow',
                    label='Destinations', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='x', color='orange',
                    label='Collision Point', markersize=10, linestyle='None'),
            Line2D([0], [0], color='gray', label='Buildings'),
            Line2D([0], [0], color='lightblue', label='Drone Trail',
                   linestyle='--')
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

    def _setup_animation(self):
        """Set up the animation timer"""
        self.ani = FuncAnimation(
            self.fig,
            self._animation_update,
            interval=self.animation_interval,
            blit=False,
            cache_frame_data=False
        )

    def _update_drone_trail(self, drone: dict) -> None:
        """Update the trail for a single drone"""
        drone_id = drone['id']
        pos = drone['position']
        
        # Add new trail segment
        if len(self.drone_trails[drone_id]) >= self.max_trail_length:
            old_line = self.drone_trails[drone_id].pop(0)
            old_line.remove()
            
        if len(self.drone_trails[drone_id]) > 0:
            last_line = self.drone_trails[drone_id][-1]
            last_x_data = last_line.get_xdata()
            last_y_data = last_line.get_ydata()
            last_pos = (last_x_data[-1], last_y_data[-1])  # Get the last point
            
            line = self.ax.plot(
                [last_pos[0], pos[0]], 
                [last_pos[1], pos[1]],
                color='white',
                alpha=self.trail_alpha,
                linewidth=1,
                zorder=2
            )[0]
            self.drone_trails[drone_id].append(line)
        else:
            # First point in trail
            line = self.ax.plot(
                [pos[0]], [pos[1]],
                color='grey',
                alpha=self.trail_alpha,
                linewidth=1,
                zorder=2
            )[0]
            self.drone_trails[drone_id].append(line)