import logging
import warnings
from pathlib import Path
import threading

import matplotlib
matplotlib.use('TkAgg')  # Use GUI backend
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from drone_sim.models.drone import DroneStatus
from drone_sim.core.system import BaseSystem
from drone_sim.core.events import EVENT_NAMES

# Configure matplotlib logging
logging.getLogger('matplotlib').setLevel(logging.ERROR)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)


class MatplotlibVisualizer(BaseSystem):
    """
    2D visualization of the drone simulation using matplotlib.
    
    Provides real-time visualization of drone flights, building layouts, and simulation metrics.
    """
    
    def __init__(self, dependencies=None):
        super().__init__(dependencies or {})
        # Basic attributes
        self.fig = None
        self.ax = None
        self.current_state = None
        self._redraw_requested = False
        self._state_lock = threading.Lock()
        self.event_bus = None
        self.engine = None
        self.config = {}
        
        # Thread and animation
        self._main_thread = threading.get_ident()
        self.animation = None
        
        # Visual elements
        self.building_patches = []
        self.drone_trails = {}   # Drone trail lines
        self.collision_artists = []
        self.drone_markers = {}
        self.info_bubbles = {}
        
        # Default configuration
        self.dimensions = (2000, 2000, 800)  # Match simulation config defaults
        self.city_name = "SimCity"
        self.show_labels = True
        self.real_time = True
        
        self.max_altitude = self.dimensions[2]
        self.max_trail_length = 50
        self.trail_alpha = 0.3
        
        self.drone_marker_styles = {
            DroneStatus.ACTIVE.value: '^',
            DroneStatus.IDLE.value: 'o',
            DroneStatus.SUCCESSFUL.value: 'P',
            DroneStatus.EMERGENCY.value: 'X',
            DroneStatus.COLLIDED.value: 'x'
        }
        self.status_colors = {
            DroneStatus.ACTIVE.value: 'green',
            DroneStatus.IDLE.value: 'blue',
            DroneStatus.SUCCESSFUL.value: 'gold',
            DroneStatus.EMERGENCY.value: 'red',
            DroneStatus.COLLIDED.value: 'black'
        }
        self.drone_colors = {}
        self.drones_scatter = {}
        
        # Process configuration from dependencies if available
        config = (dependencies or {}).get('config', {})
        sim_config = config.get('simulation', {})
        # Only update dimensions if provided in config
        if 'dimensions' in sim_config:
            self.dimensions = (
                sim_config['dimensions'].get('x', 2000),
                sim_config['dimensions'].get('y', 2000),
                sim_config['dimensions'].get('z', 800)
            )
        self.max_altitude = self.dimensions[2]
        env_config = config.get('systems', {}).get('environment', {})
        self.city_name = env_config.get('city', {}).get('name', 'SimCity')
        
        logger.debug("Initializing visualizer with dimensions: %s", self.dimensions)
        try:
            plt.style.use('dark_background')
            # Create figure and axes
            self.fig = plt.figure(figsize=(16, 9))
            self.ax = self.fig.add_subplot(111)
            plt.subplots_adjust(left=0.1, bottom=0.1, right=0.85, top=0.95)
            
            self._static_plot_setup()
            
            # Set up output directory
            self.output_dir = Path.cwd() / "output"
            self.output_dir.mkdir(exist_ok=True)
            
            self.animation_interval = 50
            plt.ion() if self.real_time else plt.ioff()
            plt.show(block=False)
            
            self._setup_animation()
            self._update_time_label()
            
            logger.debug("Visualizer initialization complete")
            logger.debug("Matplotlib backend: %s", matplotlib.get_backend())
            logger.debug("Real-time mode: %s", self.real_time)
        except Exception as e:
            logger.error("Visualizer initialization failed: %s", str(e))
            raise

    def configure(self, config: dict):
        """Dependency injection and configuration setup."""
        super().configure(config)
        self.event_bus = self.dependencies.get('event_bus')
        self.engine = self.dependencies.get('engine')
        if not self.event_bus:
            logger.error("Visualization system missing event bus reference!")
            return
        self.event_bus.subscribe(
            EVENT_NAMES['STATE_UPDATED'],
            self.on_state_update
        )
        logger.info("Subscribed to state updates on event bus")
        self._init_visualization()

    def _check_thread(self):
        if threading.get_ident() != self._main_thread:
            raise RuntimeError("Matplotlib operations must be in main thread")

    def _animation_init(self):
        """Initialize animation by returning empty list of artists"""
        return []

    def _animation_update(self, frame):
        """Main animation update function with proper blitting"""
        if not self.current_state:
            logger.warning("No state available at frame %d", frame)
            return []
        
        try:
            logger.debug("Frame %d: Updating visualization (t=%.1f, %d drones)",
                        frame,
                        self.current_state.get('time', -1),
                        len(self.current_state.get('drones', [])))
            
            # Clear old artists
            self._clear_dynamic_elements()
            
            # Update time label (non-animated)
            self._update_time_label()
            
            # Store current artists to return for blit
            artists = []
            
            # Update dynamic elements
            drone_artists = self._update_all_drones()
            artists.extend(drone_artists)
            
            collision_artists = self._update_collisions()
            artists.extend(collision_artists)
            
            # Restore the background (includes static elements and time label)
            self.fig.canvas.restore_region(self.background)
            
            # Draw all dynamic artists
            for artist in artists:
                if artist is not None:
                    self.ax.draw_artist(artist)
            
            # Update the display
            self.fig.canvas.blit(self.ax.bbox)
            self.fig.canvas.flush_events()
            
            logger.debug("Frame %d: Update complete", frame)
            return artists
            
        except Exception as e:
            logger.error("Frame %d update failed: %s", frame, e, exc_info=True)
            return []

    def _clear_dynamic_elements(self):
        # Clear dynamic drone scatter plots
        for scatter in self.drones_scatter.values():
            if scatter is not None:
                scatter.remove()
        self.drones_scatter.clear()
        # Remove info bubbles
        for bubble in self.info_bubbles.values():
            bubble.remove()
        self.info_bubbles.clear()

    def _update_all_drones(self):
        """Update all drone visualizations and return list of artists"""
        if not self.current_state:
            return []

        artists = []
        
        # Clear previous drone markers
        for marker in self.drone_markers.values():
            marker.remove()
        self.drone_markers.clear()

        for drone in self.current_state.get('drones', []):
            try:
                # Support both dict and object representations
                drone_id = drone['id'] if isinstance(drone, dict) else drone.id
                position = drone['position'] if isinstance(drone, dict) else drone.position
                status = drone['status'] if isinstance(drone, dict) else drone.status
                velocity = drone['velocity'] if isinstance(drone, dict) else drone.velocity
                flight_path = drone.get('flight_path', {}) if isinstance(drone, dict) else drone.flight_path
                
                # Plot start point and destination for each drone
                waypoints = flight_path.get('waypoints', []) if isinstance(flight_path, dict) else flight_path.waypoints
                if waypoints:
                    # Plot start point
                    start_point = self.ax.plot(waypoints[0][0], waypoints[0][1], 'bo', markersize=8, label='Start')[0]
                    artists.append(start_point)
                    
                    # Plot intermediate waypoints
                    for wp in waypoints[1:-1]:
                        wp_point = self.ax.plot(wp[0], wp[1], 'yo', markersize=6)[0]
                        artists.append(wp_point)
                        
                    # Plot destination
                    dest_point = self.ax.plot(waypoints[-1][0], waypoints[-1][1], 'y*', markersize=10, label='Destination')[0]
                    artists.append(dest_point)
                
                # Update drone markers and collect artists
                marker = self._update_drone_marker(drone_id, position, status)
                if marker:
                    artists.append(marker)
                    
                bubble = self._update_info_bubble(drone_id, position, status, velocity)
                if bubble:
                    artists.append(bubble)
                    
                trail = self._update_drone_trail(drone)
                if trail:
                    artists.append(trail)
                    
            except (KeyError, AttributeError) as e:
                logger.error("Missing drone field %s in: %s", e, drone)
                
        return artists

    def _update_drone_marker(self, drone_id, position, status):
        """Update or create drone marker with proper styling based on status"""
        try:
            if drone_id not in self.drone_markers:
                # Create new marker
                marker, = self.ax.plot(
                    position[0], position[1],
                    marker=self.drone_marker_styles.get(status, 'o'),
                    markersize=10,
                    color=self.status_colors.get(status, 'gray'),
                    alpha=0.8,
                    zorder=3,  # Ensure drones appear above buildings
                    label=f'Drone {drone_id}'
                )
                self.drone_markers[drone_id] = marker
            else:
                # Update existing marker
                marker = self.drone_markers[drone_id]
                marker.set_data(position[0], position[1])
                marker.set_color(self.status_colors.get(status, 'gray'))
                marker.set_marker(self.drone_marker_styles.get(status, 'o'))
                
            # Ensure marker is visible
            if not marker.get_visible():
                marker.set_visible(True)
                
            return marker
        except Exception as e:
            logger.error(f"Error updating drone marker {drone_id}: {e}")
            return None

    def _update_info_bubble(self, drone_id, position, status, velocity):
        """Update or create info bubble and return the artist"""
        try:
            if drone_id in self.info_bubbles:
                self.info_bubbles[drone_id].remove()
                
            info_text = (
                f"ID: {drone_id}\n"
                f"Speed: {np.linalg.norm(velocity):.1f} m/s\n"
                f"Alt: {position[2]:.1f}m"
            )
            # Add slight offset based on drone ID to prevent overlapping bubbles
            x_offset = 10 + (drone_id % 3) * 5  # Stagger horizontally
            y_offset = 10 + (drone_id % 2) * 5  # Stagger vertically
            
            bubble = self.ax.annotate(
                info_text,
                xy=(position[0], position[1]),
                xytext=(x_offset, y_offset),
                textcoords='offset points',
                bbox=dict(
                    boxstyle='round,pad=0.5',
                    fc=self.status_colors.get(status, 'blue'),
                    alpha=0.8,
                    edgecolor='white',
                    linewidth=1
                ),
                fontsize=9,
                color='white',
                zorder=4  # Ensure info bubbles appear above everything
            )
            self.info_bubbles[drone_id] = bubble
            return bubble
        except Exception as e:
            logger.error(f"Error updating info bubble for drone {drone_id}: {e}")
            return None

    def _update_drone_trail(self, drone):
        """Update or create drone trail and return the artist"""
        try:
            drone_id = drone['id'] if isinstance(drone, dict) else drone.id
            position = drone['position'] if isinstance(drone, dict) else drone.position
            status = drone['status'] if isinstance(drone, dict) else drone.status
            
            if drone_id not in self.drone_trails:
                line, = self.ax.plot(
                    [position[0]], [position[1]],
                    linestyle='--',
                    alpha=0.6,
                    color=self.status_colors.get(status, 'gray'),
                    linewidth=1.5,
                    zorder=2  # Above buildings, below drones
                )
                self.drone_trails[drone_id] = line
            else:
                line = self.drone_trails[drone_id]
                xdata, ydata = line.get_data()
                new_x = np.append(xdata, position[0])
                new_y = np.append(ydata, position[1])
                if len(new_x) > self.max_trail_length:
                    new_x = new_x[-self.max_trail_length:]
                    new_y = new_y[-self.max_trail_length:]
                line.set_data(new_x, new_y)
                # Update trail color to match current status
                line.set_color(self.status_colors.get(status, 'gray'))
            
            # Ensure trail is visible
            if not line.get_visible():
                line.set_visible(True)
                
            return line
            
        except (KeyError, AttributeError) as e:
            logger.error(f"Error updating trail for drone {drone_id}: {e}")
            return None

    def on_state_update(self, state_data: dict):
        """Handle incoming state updates with validation and proper animation handling."""
        try:
            required_keys = {'drones', 'buildings', 'collisions', 'time'}
            missing = required_keys - state_data.keys()
            if missing:
                logger.warning("Invalid state - missing keys: %s", missing)
                return
            if not all(isinstance(state_data[key], list) for key in ['drones', 'buildings', 'collisions']):
                logger.error("Invalid state types for drones, buildings, or collisions")
                return
                
            logger.debug("Processing state update: t=%.1f, %d drones",
                         state_data.get('time', -1), len(state_data['drones']))
                         
            with self._state_lock:
                # Update state without triggering redraw
                self.current_state = state_data
                
                # Let animation handle the update
                if self.animation and self.animation.event_source:
                    # Ensure animation is running at proper speed
                    self.animation.event_source.interval = 20  # 50 FPS
                else:
                    # Fallback if animation isn't running
                    self._animation_update(0)
                    
        except Exception as e:
            logger.error("Failed to process state update: %s", e)

    def _update_collisions(self):
        """Update collision markers and return list of artists"""
        if not self.current_state:
            return []
        
        artists = []
        
        # Clear existing collision markers
        for artist in self.collision_artists:
            artist.remove()
        self.collision_artists.clear()
        
        for collision in self.current_state.get('collisions', []):
            try:
                # Support both dict and object access patterns
                position = collision['position'] if isinstance(collision, dict) else collision.position
                avoided = collision.get('avoided', False) if isinstance(collision, dict) else getattr(collision, 'avoided', False)
                timestamp = collision.get('timestamp', None) if isinstance(collision, dict) else getattr(collision, 'timestamp', None)
                
                if not position:
                    logger.warning("Collision record missing position data")
                    continue
                    
                if avoided:
                    marker = self.ax.scatter(
                        position[0], position[1],
                        s=100, c='lime', marker='o',
                        edgecolors='black', linewidths=1.5,
                        alpha=0.7, zorder=4
                    )
                    label = f"Avoided\n{timestamp:.1f}s" if timestamp is not None else "Avoided"
                    text = self.ax.text(
                        position[0], position[1] + 15,
                        label,
                        color='lime', ha='center', fontsize=8,
                        bbox=dict(facecolor='black', alpha=0.5)
                    )
                    self.collision_artists.extend([marker, text])
                    artists.extend([marker, text])
                else:
                    marker = self.ax.scatter(
                        position[0], position[1],
                        s=100, c='red', marker='X',
                        alpha=0.7, zorder=4
                    )
                    self.collision_artists.append(marker)
                    artists.append(marker)
                    
            except (KeyError, AttributeError) as e:
                logger.error("Invalid collision data format: %s", e)
                continue
                
        return artists

    def save_plot(self) -> None:
        """Save the current plot state to a file."""
        if not self.current_state:
            logger.warning("No simulation state available for saving")
            return
        if self.fig is None or not plt.fignum_exists(self.fig.number) or not hasattr(self.fig.canvas, 'get_renderer'):
            logger.debug("Cannot save plot - figure closed or not initialized")
            return
        try:
            self.output_dir.mkdir(exist_ok=True, parents=True)
            save_path = self.output_dir / f"sim_state_{self.current_state['time']:.1f}s.png"
            export_path = str(save_path.resolve())
            self.fig.canvas.draw()
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                self.fig.savefig(export_path, bbox_inches='tight')
            logger.info(f"Saved simulation state to: {export_path}")
        except RuntimeError as e:
            if "closed figure" in str(e):
                logger.debug("Save attempted on closed figure")
            else:
                logger.error(f"Matplotlib error: {e}")
        except Exception as e:
            logger.error(f"Error saving plot: {e}")

    def _static_plot_setup(self):
        """
        Set up static aspects of the plot (axes limits, grid, labels, and legend).
        """
        # Add more padding (100m) to ensure all markers and info bubbles are visible
        self.ax.set_xlim(-100, self.dimensions[0] + 100)
        self.ax.set_ylim(-100, self.dimensions[1] + 100)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_title("Drone Flight Simulation")
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        # Create legend elements that match actual marker styles
        legend_elements = []
        
        # Add drone status markers
        for status in DroneStatus:
            marker_style = self.drone_marker_styles.get(status.value, 'o')
            color = self.status_colors.get(status.value, 'gray')
            legend_elements.append(
                Line2D([0], [0], marker=marker_style, color='w',
                      markerfacecolor=color, label=f'{status.label.title()} Drones',
                      markersize=10, linestyle='None')
            )
        
        # Add other elements
        legend_elements.extend([
            Line2D([0], [0], marker='o', color='blue', label='Start Points',
                  markersize=8, linestyle='None'),
            Line2D([0], [0], marker='*', color='yellow', label='Destinations',
                  markersize=10, linestyle='None'),
            Line2D([0], [0], marker='X', color='red', label='Collision Points',
                  markersize=10, linestyle='None'),
            Line2D([0], [0], color='gray', label='Buildings'),
            Line2D([0], [0], color='lightblue', label='Drone Trail',
                  linestyle='--'),
            Line2D([0], [0], marker='o', color='lime',
                  label='Avoided Collisions', markersize=10, linestyle='None')
        ])
        self.ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=9)

    def _update_time_label(self):
        """Update the simulation time display."""
        try:
            sim_time = self.current_state['time'] if self.current_state else 0.0
            time_text = f"Time: {sim_time:.1f}s"
            if hasattr(self, 'time_label') and self.time_label is not None:
                self.time_label.remove()
            
            # Create new time label without animation for better visibility
            self.time_label = self.fig.text(
                0.02, 0.98,  # Move to top-left corner
                time_text,
                fontsize=12,
                color='white',
                bbox=dict(facecolor='black', alpha=0.7, edgecolor='white', linewidth=1),
                verticalalignment='top'  # Align to top
            )
            # Force immediate draw of time label
            self.fig.canvas.draw()
        except Exception as e:
            logger.error(f"Error updating time label: {e}")

    def _setup_animation(self):
        """Set up the main animation with proper blitting and background saving."""
        self._check_thread()
        if self.animation:
            logger.debug("Stopping previous animation")
            self.animation.event_source.stop()
            self.animation._stop()
            self.animation = None
            
        # Draw static elements first
        self._static_plot_setup()
        self._create_static_elements()
        
        # Update time label (it's now static)
        self._update_time_label()
        
        # Save the complete background including static elements
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        
        # Set up animation with faster interval and proper blitting
        self.animation = FuncAnimation(
            self.fig,
            self._animation_update,
            init_func=self._animation_init,
            interval=20,  # 50 FPS for smoother animation
            blit=True,
            cache_frame_data=False
        )
        
        # Enable interactive mode
        plt.ion()
        logger.debug("Animation initialized with interval 20ms (50 FPS)")

    def _init_visualization(self):
        """Initialize core visualization components."""
        self._check_thread()
        try:
            if self.fig is None or not plt.fignum_exists(self.fig.number):
                logger.debug("Creating new figure")
                self.fig = plt.figure(figsize=(16, 9))
                self.ax = self.fig.add_subplot(111)
                plt.subplots_adjust(left=0.1, bottom=0.1, right=0.85, top=0.95)
            else:
                logger.debug("Using existing figure")
            self._setup_axes()
            self._create_static_elements()
            logger.debug("Visualization initialized")
        except Exception as e:
            logger.error("Visualization init failed: %s", e)
            raise

    def _setup_axes(self):
        """Initialize 2D axes using config values."""
        self._check_thread()
        if self.ax is not None and self.ax in self.fig.axes:
            self.fig.delaxes(self.ax)
        self.ax = self.fig.add_subplot(111)
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.85, top=0.95)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        # Use same padding as _static_plot_setup for consistency
        self.ax.set_xlim(-100, self.dimensions[0] + 100)
        self.ax.set_ylim(-100, self.dimensions[1] + 100)
        self.ax.text(
            self.dimensions[0] / 2, self.dimensions[1] + 20,
            self.city_name,
            ha='center', va='center', fontsize=12,
            bbox=dict(facecolor='white', alpha=0.8)
        )

    def _create_static_elements(self):
        """Create permanent visualization elements."""
        self._check_thread()
        self._static_plot_setup()
        if self.engine and hasattr(self.engine.systems['environment'], 'current_city'):
            buildings = [b.to_dict() for b in self.engine.systems['environment'].current_city.buildings]
            self._create_building_patches(buildings)
        logger.debug("Static elements initialized")

    def _create_building_patches(self, buildings: list):
        """Create building rectangles with height‐based shading."""
        self._check_thread()
        try:
            for b in buildings:
                height_norm = b['height'] / self.max_altitude
                color = str(0.5 + height_norm * 0.5)  # greyscale between 0.5 and 1.0
                rect = plt.Rectangle(
                    (b['x'], b['y']),
                    b['width'],
                    b['length'],
                    facecolor=color,
                    edgecolor='0.2',
                    alpha=0.7,
                    zorder=1
                )
                self.ax.add_patch(rect)
                self.building_patches.append(rect)
            logger.debug("Created %d building patches", len(buildings))
        except Exception as e:
            logger.error("Failed to create building patches: %s", e)
            raise

    def close(self):
        """Properly close visualization resources."""
        logger.debug("Closing visualization resources...")
        try:
            if self.animation:
                logger.debug("Stopping animation")
                if self.animation.event_source:
                    self.animation.event_source.stop()
                if hasattr(self.animation, '_resize_id'):
                    self.animation._stop()
        except AttributeError as e:
            logger.debug("Animation cleanup error: %s", e)
        finally:
            self.animation = None
            if self.fig and plt.fignum_exists(self.fig.number):
                plt.close(self.fig)
            self.fig = None

    def update(self, dt: float):
        """Required by simulation engine for system updates (handled by animation)."""
        pass

    def start_gui_loop(self):
        """Ensure the GUI loop runs in the main thread."""
        if threading.current_thread() != threading.main_thread():
            logger.warning("GUI loop must run in main thread - use plt.show() instead")
            return
        logger.debug("Starting GUI main loop in main thread")
        plt.show(block=True)
