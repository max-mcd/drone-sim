from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class CollisionRecord:
    drone_ids: Tuple[str, str]
    position: np.ndarray
    severity: float
    timestamp: float
    resolution: str
    avoided: bool 