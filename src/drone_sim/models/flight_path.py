from typing import Optional

import numpy as np

import logging

logger = logging.getLogger(__name__)


class FlightPath:
    """Manages a sequence of waypoints for a drone's flight.

    A flight path consists of ordered 3D waypoints that a drone must visit.
    The path includes:
    - Start point (first waypoint)
    - Optional intermediate waypoints
    - Destination (final waypoint)
    
    Waypoint Navigation:
    - Drones fly directly toward their current waypoint
    - A waypoint is considered "reached" when within threshold distance
    - Speed reduces gradually when approaching waypoints
    - After reaching a waypoint, drone advances to next in sequence

    """
        
    def __init__(self, waypoints: list):
        """Initialize flight path with waypoints
        
        Args:
            waypoints: List of waypoint coordinates [x, y, z]
        """
        if not waypoints:
            raise ValueError("Waypoints list cannot be empty")
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints required")
        if not all(isinstance(w, np.ndarray) and w.shape == (3,) for w in waypoints):
            raise ValueError("All waypoints must be 3D numpy arrays")
        self.waypoints = [np.array(wp) for wp in waypoints]
        self.current_target_index = 0
        
    @property
    def current_target(self):
        return self.waypoints[self.current_target_index]
    
    @property
    def current_waypoint(self) -> np.ndarray:
        """Get current waypoint"""
        return self.waypoints[self.current_target_index]
    
    @property
    def next_waypoint(self) -> Optional[np.ndarray]:
        """Get next waypoint if available"""
        if self.current_target_index + 1 < len(self.waypoints):
            return self.waypoints[self.current_target_index + 1]
        return None
        
    def is_nearing_waypoint(self, position: np.ndarray, 
                           horizontal_threshold: float = 2.0,
                           vertical_threshold: float = 5.0) -> bool:
        horizontal_dist = np.linalg.norm(position[:2] - self.current_waypoint[:2])
        vertical_dist = abs(position[2] - self.current_waypoint[2])
        return horizontal_dist < horizontal_threshold and vertical_dist < vertical_threshold
        
    def advance_waypoint(self) -> bool:
        """Move to next waypoint if available
        
        Returns:
            False if at final waypoint, True otherwise
        """
        if self.current_target_index + 1 < len(self.waypoints):
            self.current_target_index += 1
            return True
        return False