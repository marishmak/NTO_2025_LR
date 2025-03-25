from enum import Enum

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


class DroneState(BaseModel):
    ready_to_land: bool
