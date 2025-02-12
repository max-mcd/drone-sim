from typing import Dict, Protocol, runtime_checkable

import numpy as np


class SerializableEntity(Protocol):
    """Protocol for model serialization"""
    def to_dict(self) -> Dict: ... 

@runtime_checkable
class Collidable(Protocol):
    """Protocol for objects that can collide"""
    position: tuple[float, float, float]
    @property
    def collision_radius(self) -> float: ...
    
    def predict_position(self, time: float) -> np.ndarray: ...

@runtime_checkable
class Renderable(Protocol):
    """Protocol for objects that can be visualized"""
    def get_visual_state(self) -> dict:
        """Return visualization state"""
        ...

@runtime_checkable
class Updatable(Protocol):
    """Protocol for systems that need regular updates"""
    def update(self, dt: float) -> None: ...

@runtime_checkable
class Configurable(Protocol):
    def configure(self, config: dict) -> None: ... 