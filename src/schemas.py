from enum import Enum
from typing import List, Tuple

from pydantic import BaseModel


class Action(str, Enum):
    start_mission = "start_mission"
    emergency_shutdown = "emergency_shutdown"
    land = "land"
    pause = "pause"


class ButtonAction(BaseModel):
    action: Action


class Message(BaseModel):
    message: str


class Status(BaseModel):
    connection: bool
    readiness: bool
    battery_voltage: float
    battery_percent: float


class DroneState(BaseModel):
    ready_to_start: bool
    ready_to_land: bool


class ImageCoord(BaseModel):
    coords: List[Tuple[float, float, float]]


class FireData(BaseModel):
    image_data: str
    coordinates: Tuple[float, float]
    area: float
