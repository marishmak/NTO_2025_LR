from fastapi import HTTPException, status
from schemas import DroneState


class DroneStateRepository:
    def __init__(self):
        self.drone_states = {
            0: DroneState(ready_to_land=False, second_got_to_14=False, ready_to_start=False),
            1: DroneState(ready_to_land=False, second_got_to_14=False, ready_to_start=False),
        }

    def get_state(self, drone_id: int) -> DroneState:
        if drone_id not in self.drone_states:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Drone {drone_id} not available",
            )
        return self.drone_states[drone_id]

    def update_state(self, drone_id: int, new_state: DroneState):
        if drone_id not in self.drone_states:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Drone {drone_id} not available",
            )
        self.drone_states[drone_id] = new_state


drone_state_repo = DroneStateRepository()
