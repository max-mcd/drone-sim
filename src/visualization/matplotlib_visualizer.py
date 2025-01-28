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

6. Legend Placement
   - Move legend outside the plot grid to the right side
   - Update __init__ to adjust figure size and subplot parameters:
     self.fig = plt.figure(figsize=(16, 8))  # Wider figure to accommodate legend
     plt.subplots_adjust(right=0.85)  # Leave space for legend
     self.ax.legend(handles=self.legend_elements, 
                   loc='center left', 
                   bbox_to_anchor=(1.05, 0.5))

7. Simulation Time Display
   - Add time display in lower left corner
   - Update _animation_update:
     self.ax.text(0.02, 0.02, 
                  f'Simulation Time: {state.time:.1f}s',
                  transform=self.ax.transAxes,
                  fontsize=10,
                  bbox=dict(facecolor='black', alpha=0.7))

8. Success Indication
   - Update _plot_drone to use green for successful drones:
     if drone['status'] == 'successful':
         marker = 'o'  # Circle for successful drones
         color = 'green'
         bubble_color = 'green'  # Change from yellow to green
         bubble_text_color = 'white'  # Better contrast on green

9. Additional Ideas:
   - Add completion percentage
   - Add collision counter
   - Add average speed indicator
   - Add mission success/failure ratio
   - Add elapsed real time vs simulation time
"""

import logging
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from ..models.simulation_state import SimulationState

# Configure logging after imports but before any matplotlib usage
logging.getLogger('matplotlib').setLevel(logging.ERROR)
logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.ERROR)

# Suppress matplotlib warnings and debug messages
plt.set_loglevel('warning')

# Initialize logger for this module
logger = logging.getLogger(__name__)


class MatplotlibVisualizer:
    """2D visualization of the drone simulation using matplotlib"""
    
    def __init__(self, dimensions: Tuple[float, float, float], real_time: bool = True):
        """Initialize the visualizer with given dimensions and mode
        
        Args:
            dimensions: (width, length, height) of simulation space
            real_time: If True, updates display in real-time. If False, runs faster
        """
        # Setup logging first
        self.logger = logging.getLogger(__name__)
        
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
        
        # Create figure and axes
        self.fig = plt.figure(figsize=(16, 8))
        self.ax = self.fig.add_subplot(111)
        plt.subplots_adjust(right=0.85)
        
        # Setup storage
        self.output_dir = Path.cwd() / "output"
        self.output_dir.mkdir(exist_ok=True)
        self.drone_trails = {}
        self.max_trail_length = 50
        self.current_state = None
        
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
        
        # Setup state management - increase buffer size and add frame skip
        self.max_buffer_size = 500  # Increased from 100
        self.frame_skip = 2 if real_time else 1  # Skip frames in real-time mode
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
        
        # Set matplotlib mode
        if real_time:
            plt.ion()
        else:
            plt.ioff()
        
        self.logger.debug("Visualizer initialization complete")
        
    def setup_plot(self) -> None:
        """Initialize plot styling and axes"""
        self.logger.debug("Setting up plot...")
        self.ax.set_xlim(-50, self.dimensions[0] + 50)
        self.ax.set_ylim(-50, self.dimensions[1] + 50)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_title("Drone Flight Simulation")
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.legend(handles=self.legend_elements, 
                      loc='center left', 
                      bbox_to_anchor=(1.05, 0.5))
        self.logger.debug("Plot setup complete")
        
    def _animation_update(self, frame):
        """Update function called by animation system"""
        if not self.animation_started:
            self.animation_started = True
            self.logger.info("Animation loop started")
        
        self.frame_count += 1
        self.logger.debug(f"Animation frame {self.frame_count}")
        
        # Verify state availability
        if not self.state_buffer and not self.last_state:
            self.logger.warning("No states available for animation")
            return
        
        # Get current state with frame skipping
        if not self.state_buffer:
            state = self.last_state
            self.logger.debug("Using last state (buffer empty)")
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
            self.logger.error("Invalid state object")
            return
        
        self.logger.debug(f"State contains: {len(state.drones)} drones, {len(state.buildings)} buildings")
        
        # Store current state for use in _plot_drone
        self.current_state = state
        
        # Clear and setup
        self.ax.clear()
        self.setup_plot()
        
        # Add stored building patches
        for patch in self.building_patches:
            self.ax.add_patch(patch)
        
        # Draw drones with verification
        for drone in state.drones:
            try:
                self.logger.debug(f"Drawing drone {drone['id']} at {drone['position']}")
                
                # Verify position data
                if not all(isinstance(x, (int, float)) for x in drone['position']):
                    self.logger.error(f"Invalid position data for drone {drone['id']}")
                    continue
                
                # Update and draw trail
                self._update_trails(drone)
                
                # Draw drone and all its components
                self._plot_drone(drone)
                
            except Exception as e:
                self.logger.error(f"Failed to draw drone {drone['id']}: {e}")
        
        # Plot collision points - only show collisions that have happened up to current time
        current_collisions = [
            collision for collision in (state.drone_collisions + state.building_collisions)
            if collision[2] <= state.time  # collision[2] is the collision time
        ]
        self._plot_collisions(current_collisions)
        
        # Add simulation time display (once per frame)
        time_text = f'Simulation Time: {state.time:.1f}s'
        if all(drone['status'] in ['successful', 'collided'] for drone in state.drones):
            time_text += ' (COMPLETE)'
            if self.on_simulation_complete:
                callback = self.on_simulation_complete
                self.on_simulation_complete = None  # Prevent multiple calls
                callback(state.time)
        
        self.ax.text(0.02, 0.02, 
                    time_text,
                    transform=self.ax.transAxes,
                    fontsize=10,
                    bbox=dict(facecolor='black', alpha=0.7))
        
        self.frames_rendered += 1
        self.logger.debug(f"Frame {self.frame_count} complete. Total frames rendered: {self.frames_rendered}")

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
        """Buffer state updates from simulation"""
        if not self.received_first_state:
            self.received_first_state = True
            self.logger.info("Received first state update")
            self._initialize_building_patches(state.buildings)
        
        self.state_updates_received += 1
        
        # Skip some updates if buffer is getting full
        if len(self.state_buffer) > self.max_buffer_size * 0.8:  # 80% full
            if self.state_updates_received % 3 != 0:  # Skip two out of three updates
                self.logger.debug("Skipping update due to high buffer usage")
                return
        
        # Add state to buffer
        if len(self.state_buffer) < self.max_buffer_size:
            self.state_buffer.append(state)
            self.logger.debug(f"""
                State update received:
                Update #{self.state_updates_received}
                Time: {state.time:.2f}s
                Drones: {len(state.drones)}
                Buildings: {len(state.buildings)}
                Buffer size: {len(self.state_buffer)}/{self.max_buffer_size}
            """)
        else:
            self.logger.warning(
                f"State buffer full ({self.max_buffer_size}), dropping update. "
                f"Consider increasing animation_interval or reducing simulation speed."
            )

    def _initialize_building_patches(self, buildings) -> None:
        """Create building patches once during initialization"""
        self.logger.debug("Initializing building patches...")
        self.building_patches = []
        
        for i, building in enumerate(buildings):
            try:
                rect = plt.Rectangle(
                    (building.x - building.width/2,
                     building.y - building.length/2),
                    building.width,
                    building.length,
                    color='gray',
                    alpha=0.5,
                    zorder=1
                )
                self.building_patches.append(rect)
            except Exception as e:
                self.logger.error(f"Failed to create building patch {i}: {e}", exc_info=True)
        
        self.logger.debug(f"Created {len(self.building_patches)} building patches")

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
                self.logger.error("Cannot save plot: Figure no longer exists")
                return
                
            # Force a redraw of the figure
            self.fig.canvas.draw()
            
            # Save the figure
            save_path = self.output_dir / filename
            self.fig.savefig(save_path)
            print(f"\nFinal state saved to: {save_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to save plot: {e}", exc_info=True)

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