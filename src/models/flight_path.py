from typing import List, Optional

import numpy as np


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
        
    def __init__(self, waypoints: List[np.ndarray]) -> None:
        """Initialize flight path with waypoints
        
        Args:
            waypoints: List of waypoint coordinates [x, y, z]
        """
        if not waypoints:
            raise ValueError("Waypoints list cannot be empty")
        if len(waypoints) < 2:
            raise ValueError("Flight path must have at least 2 waypoints")
        if not all(isinstance(w, np.ndarray) and w.shape == (3,) for w in waypoints):
            raise ValueError("All waypoints must be 3D numpy arrays")
            
        self.waypoints = waypoints
        self.current_index = 0
        
    @property
    def current_waypoint(self) -> np.ndarray:
        """Get current waypoint"""
        return self.waypoints[self.current_index]
    
    @property
    def next_waypoint(self) -> Optional[np.ndarray]:
        """Get next waypoint if available"""
        if self.current_index + 1 < len(self.waypoints):
            return self.waypoints[self.current_index + 1]
        return None
        
    def is_nearing_waypoint(self, position: np.ndarray, threshold: float = 1.0) -> bool:
        """Check if position is close to current waypoint
        
        Args:
            position: Current position to check
            threshold: Distance threshold in meters
            
        Returns:
            True if within threshold distance of waypoint
        """
        return np.linalg.norm(position - self.current_waypoint) < threshold
        
    def advance_waypoint(self) -> bool:
        """Move to next waypoint if available
        
        Returns:
            False if at final waypoint, True otherwise
        """
        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
            return True
        return False