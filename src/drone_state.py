from fastapi import HTTPException, status

from schemas import DroneState


class DroneStateRepository:
    """Repository for managing drone states in memory.

    Maintains a dictionary of drone states indexed by drone_id.
    """

    def __init__(self):
        """Initializes repository with default states for drones 0 and 1."""
        self.drone_states = {
            0: DroneState(
                ready_to_land=False, second_got_to_14=False, ready_to_start=False
            ),
            1: DroneState(
                ready_to_land=False, second_got_to_14=False, ready_to_start=False
            ),
        }

    def get_state(self, drone_id: int) -> DroneState:
        """Retrieves the state for a specific drone.

        Args:
            drone_id: The ID of the drone.

        Returns:
            DroneState object for the specified drone.

        Raises:
            HTTPException (404): If drone_id is not found.
        """
        if drone_id not in self.drone_states:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Drone {drone_id} not available",
            )
        return self.drone_states[drone_id]

    def update_state(self, drone_id: int, new_state: DroneState):
        """Updates the state for a specific drone.

        Args:
            drone_id: The ID of the drone.
            new_state: New DroneState to be set.

        Raises:
            HTTPException (404): If drone_id is not found.
        """
        if drone_id not in self.drone_states:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Drone {drone_id} not available",
            )
        self.drone_states[drone_id] = new_state


drone_state_repo = DroneStateRepository()
