import time
from collections import defaultdict
from typing import Any, Callable

from drone_sim.core.simulation_state import SimulationState

import logging

logger = logging.getLogger(__name__)


class EventBus:
    def __init__(self):
        self.subscriptions = defaultdict(list)
        self._universal_subscribers = []  # For subscribe_all
        
    def publish(self, event_name: str, data: dict = None):
        """Publish event to all subscribers"""
        # Notify specific subscribers
        for callback in self.subscriptions[event_name]:
            try:
                callback(data)
            except Exception as e:
                logger.error("Error in event handler for %s: %s", event_name, e)
        
        # Notify universal subscribers
        for callback in self._universal_subscribers:
            try:
                callback(event_name, data)
            except Exception as e:
                logger.error("Universal event callback failed: %s", e)

    def subscribe(self, event_name: str, callback: Callable):
        logger.debug("Subscribing to %s. Callback: %s", event_name, callback.__qualname__)
        self.subscriptions[event_name].append(callback)

    def subscribe_all(self, callback: Callable):
        """New method for universal event logging"""
        self._universal_subscribers.append(callback)

EVENT_NAMES = {
    'COLLISION_DETECTED': 'collision_detected',
    'PATH_COMPLETED': 'path_completed',
    'SIMULATION_START': 'simulation_start',
    'SIMULATION_END': 'simulation_end',
    'SYSTEM_ERROR': 'system_error',
    'MOVEMENT_UPDATED': 'movement_updated',
    'STATE_UPDATED': 'state_updated',
    'COLLISION_AVOIDED': 'collision_avoided'
} 
