from abc import ABC, abstractmethod
from typing import List

from drone_sim.models.collision import CollisionRecord


class AbstractStrategy(ABC):
    """Base class for collision avoidance strategies"""
    
    @abstractmethod
    def resolve(self, collisions) -> list:
        """Resolve detected collisions"""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Strategy identifier"""
        pass


class AvoidanceStrategy(ABC):
    @abstractmethod
    def resolve(self, collisions: List[CollisionRecord]) -> List[CollisionRecord]:
        pass 