"""2D visualization of the drone simulation using matplotlib.

Visualization features:
1. Real-time drone position and status tracking
2. Building layout visualization
3. Collision detection visualization
4. Flight path and waypoint tracking
5. Performance metrics display
6. Dynamic legend and status indicators
"""

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

# Suppress matplotlib warnings and debug messages
plt.set_loglevel('warning')

class MatplotlibVisualizer:
    """2D visualization of the drone simulation using matplotlib"""
    
    def __init__(self, dimensions: Tuple[float, float, float], real_time: bool = True):
        """Initialize the visualizer with given dimensions and mode
        
        Args:
            dimensions: (width, length, height) of simulation space
            real_time: If True, updates display in real-time. If False, runs faster
        """
        
        # Initialize counters and flags
        self.frame_count = 0
        self.received_first_state = False
        self.animation_started = False
        self.state_updates_received = 0
        self.frames_rendered = 0
        
        # Store basic parameters
        plt.style.use('dark_background')
        self.real_time = real_time
        self.dimensions = dimensions
        
        # Store building patches
        self.building_patches = []
        
        # Create figure and axes with adjusted size and margins
        self.fig = plt.figure(figsize=(16, 9))  # Taller figure
        self.ax = self.fig.add_subplot(111)
        plt.subplots_adjust(
            left=0.1,      # Less space on left
            bottom=0.1,    # Space for time display
            right=0.85,    # More space on right for legend
            top=0.95       # Use more of top
        )
        
        # Setup storage
        self.output_dir = Path.cwd() / "output"
        self.output_dir.mkdir(exist_ok=True)
        self.drone_trails = {}
        self.max_trail_length = 50
        self.current_state = None
        
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
        self.animation_interval = 50 if real_time else 1
        
        self.state_buffer = []
        self.last_state = None
        
        # Animation settings - adjust interval based on mode
        self.animation_interval = 100 if real_time else 1  # Increased from 50ms to 100ms
        
        # Add completion callback
        self.on_simulation_complete = None
        
        # Add completion flag
        self.completion_pending = False
        self.completion_time = None
        
        # Initialize plot
        self.setup_plot()
        
        # Initialize animation property
        self.ani = None
        
        # Create animation
        self.ani = FuncAnimation(
            self.fig,
            self._animation_update,
            interval=self.animation_interval,
            blit=False,
            cache_frame_data=False
        )
        
        # Force first draw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Set matplotlib mode
        if real_time:
            plt.ion()
        else:
            plt.ioff()
        
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
        """Update function called by animation system"""
        if not self.animation_started:
            self.animation_started = True
            logger.info("Animation loop started")
        
        self.frame_count += 1
        
        # Verify state availability - only warn once during startup
        if not self.state_buffer and not self.last_state:
            if not self.animation_started:
                logger.debug("Waiting for first simulation state...")  # Changed to debug level
            return
        
        # Get current state with frame skipping
        if not self.state_buffer:
            state = self.last_state
            logger.debug("Using last state (buffer empty)")
        else:
            # Skip frames if buffer is getting full
            skip_count = min(
                self.frame_skip,
                len(self.state_buffer) - 1
            ) if len(self.state_buffer) > self.frame_skip else 0
            
            # Pop multiple states but keep the last one
            for _ in range(skip_count):
                self.state_buffer.pop(0)
            
            state = self.state_buffer.pop(0)
            self.last_state = state
        
        # Verify state data
        if not state:
            logger.error("Invalid state object")
            return
        
        logger.debug(f"State contains: {len(state.drones)} drones, {len(state.buildings)} buildings")
        
        # Store current state for use in _plot_drone
        self.current_state = state
        
        # Clear and setup
        self.ax.clear()
        self.setup_plot()
        
        # Add building patches
        for patch in self.building_patches:
            self.ax.add_patch(patch)
        
        # Draw drones with verification
        for drone in state.drones:
            try:
                logger.debug(f"Drawing drone {drone['id']} at {drone['position']}")
                
                # Verify position data
                if not all(isinstance(x, (int, float)) for x in drone['position']):
                    logger.error(f"Invalid position data for drone {drone['id']}")
                    continue
                
                # Update and draw trail
                self._update_trails(drone)
                
                # Draw drone and all its components
                self._plot_drone(drone)
                
            except Exception as e:
                logger.error(f"Failed to draw drone {drone['id']}: {e}")
        
        # Plot collision points - only show collisions that have happened up to current time
        current_collisions = [
            collision for collision in (state.drone_collisions + state.building_collisions)
            if collision[2] <= state.time  # collision[2] is the collision time
        ]
        self._plot_collisions(current_collisions)
        
        # Move simulation time text below plot
        time_text = f'Simulation Time: {state.time:.1f}s'
        if all(drone['status'] in ['successful', 'collided'] for drone in state.drones):
            time_text += ' (COMPLETE)'
            if self.on_simulation_complete:
                callback = self.on_simulation_complete
                self.on_simulation_complete = None
                callback(state.time)
        
        # Use figure coordinates instead of axes coordinates
        self.fig.text(
            0.02, 0.02,  # Position in figure coordinates
            time_text,
            fontsize=10,
            color='white',  # Make text white to match dark theme
            bbox=dict(
                facecolor='black',
                alpha=0.7,
                pad=0.5,
                edgecolor='none'  # Remove the border
            )
        )
        
        self.frames_rendered += 1
        logger.debug(f"Frame {self.frame_count} complete. Total frames rendered: {self.frames_rendered}")

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

    def on_state_update(self, state: SimulationState) -> None:
        """Handle new simulation state update"""
        # Initialize buildings on first state
        if not self.received_first_state:
            self.received_first_state = True
            self._initialize_building_patches(state.buildings)
        
        self.state_buffer.append(state)
        
        # Force canvas update in fast mode
        if not self.real_time:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

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

    def save_plot(self, filename: str = "final_state.png") -> None:
        """Save current plot state to file"""
        try:
            # Make sure we have a valid figure before saving
            if not plt.fignum_exists(self.fig.number):
                logger.error("Cannot save plot: Figure no longer exists")
                return
                
            # Force a redraw of the figure
            self.fig.canvas.draw()
            
            # Save the figure
            save_path = self.output_dir / filename
            self.fig.savefig(save_path)
            print(f"\nFinal state saved to: {save_path}")
            
        except Exception as e:
            logger.error(f"Failed to save plot: {e}", exc_info=True)

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
        
        # Plot start and end points using flight path
        waypoints = np.array(drone['flight_path'])
        self.ax.scatter(
            waypoints[0][0], waypoints[0][1],  # First waypoint
            c='blue', marker='o', s=100, zorder=2
        )
        self.ax.scatter(
            waypoints[-1][0], waypoints[-1][1],  # Last waypoint
            c='yellow', marker='*', s=100, zorder=2
        )
        
        # Plot complete flight path
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
            marker = '*'  # Star for collided drones
            color = 'red'
            bubble_color = 'red'
            bubble_text_color = 'white'
        elif drone['status'] == 'successful':
            marker = 'o'  # Circle for successful drones
            color = 'green'
            bubble_color = 'green'
            bubble_text_color = 'white'
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

    def register_completion_callback(self, callback):
        """Register a callback to be called when simulation completes"""
        self.on_simulation_complete = callback 