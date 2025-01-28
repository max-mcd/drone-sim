from typing import Callable, List

from ..models.simulation_state import SimulationState


class SimulationStateManager:
    """Manages simulation state and notifies observers of changes"""
    
    def __init__(self):
        self.current_state: SimulationState = None
        self.observers: List[Callable[[SimulationState], None]] = []
    
    def update_state(self, new_state: SimulationState) -> None:
        """Update current state and notify all observers"""
        self.current_state = new_state
        self._notify_observers()
    
    def add_observer(self, observer: Callable[[SimulationState], None]) -> None:
        """Add a new observer to be notified of state changes"""
        self.observers.append(observer)
    
    def _notify_observers(self) -> None:
        """Notify all observers with current state"""
        for observer in self.observers:
            observer(self.current_state) 