"""
interfaces.py

Shared data contracts for the Adaptive Path Planning and Collision Avoidance
project.

This module defines the dataclasses used to pass state between the team's
modules (Perception, Planning, Localization, Control, Simulation,
Integration). Every module should import these types from here rather than
defining its own copies, so that data passed between modules stays
compatible across branches.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Obstacle:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    obj_type: str
    confidence: float = 1.0


@dataclass
class EgoState:
    x: float
    y: float
    theta: float
    v: float


@dataclass
class WorldState:
    ego: EgoState
    obstacles: List[Obstacle] = field(default_factory=list)
    road_boundary: List[Tuple[float, float]] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class PerceptionOutput:
    detected_obstacles: List[Obstacle] = field(default_factory=list)
    road_edges: List[Tuple[float, float]] = field(default_factory=list)
    confidence: float = 1.0
    ego_position_hint: Optional[EgoState] = None
    timestamp: float = 0.0


@dataclass
class Waypoint:
    x: float
    y: float
    target_speed: float = 0.0


@dataclass
class Path:
    waypoints: List[Waypoint] = field(default_factory=list)
    is_replanned: bool = False
    replan_reason: Optional[str] = None
    valid: bool = True