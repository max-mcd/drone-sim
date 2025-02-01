import logging
from typing import Callable, List, Protocol

from ..models.simulation_state import SimulationState

logger = logging.getLogger(__name__)


class StateObserver(Protocol):
    """Protocol for state observers.
    
    This protocol defines the interface that state observers must implement to receive
    state updates from the SimulationStateManager. Observers are notified whenever
    the simulation state changes and receive the new state object.
    
    Required Methods:
        on_state_update: Called when state changes with new SimulationState object
    """
    def on_state_update(self, state: SimulationState) -> None: ...


class SimulationStateManager:
    """Manages simulation state and notifies observers of changes using observer pattern."""
    
    def __init__(self) -> None:
        self.current_state: SimulationState | None = None
        self.observers: List[StateObserver] = []
    
    def update_state(self, state: SimulationState) -> None:
        """Update current state and notify all observers"""
        logger.debug(f"State manager received update, notifying {len(self.observers)} observers")
        self.current_state = state
        self._notify_observers()
    
    def add_observer(self, observer: Callable[[SimulationState], None]) -> None:
        """Add a new observer to be notified of state changes"""
        self.observers.append(observer)
    
    def _notify_observers(self) -> None:
        """Notify all observers with current state"""
        logger.debug(f"Notifying {len(self.observers)} observers with state at time {self.current_state.time}")
        for observer in self.observers:
            observer(self.current_state) 