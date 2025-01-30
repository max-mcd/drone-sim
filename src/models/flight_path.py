from typing import List, Optional

import numpy as np


class FlightPath:
    """Manages a sequence of waypoints for a drone's flight"""
    
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
        
    def get_next_waypoint(self) -> np.ndarray:
        """Get the next waypoint in the path"""
        return self.waypoints[self.current_index]
        
    def advance_waypoint(self) -> bool:
        """Advance to next waypoint if available
        
        Returns:
            True if advanced to next waypoint, False if at end of path
        """
        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
            return True
        return False 