import asyncio
import base64
import io
from contextlib import asynccontextmanager
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, status
from fastapi.middleware.cors import CORSMiddleware
from paramiko import SSHClient
from PIL import Image
from pymavlink import mavutil
from scp import SCPClient

from clusterize import clusterize_coords
from config import config
from drone_state import drone_state_repo
from schemas import (
    Action,
    ButtonAction,
    DroneState,
    Message,
    Status,
)
from vision import main_fire_detection

dir_path = Path(__file__).parent.parent


def connect(ssh0: SSHClient, ssh1: SSHClient):
    ssh0.load_system_host_keys()
    ssh0.connect(
        config.drone_ip0,
        config.drone_port0,
        config.drone_username,
        config.drone_password,
    )
    if not config.is_one_drone:
        ssh1.load_system_host_keys()
        ssh1.connect(
            config.drone_ip1,
            config.drone_port1,
            config.drone_username,
            config.drone_password,
        )


def disconnect(ssh0: SSHClient, ssh1: SSHClient):
    ssh0.close()
    if not config.is_one_drone:
        ssh1.close()


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield


app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

pid0 = None
pid1 = None
coords_0 = []
frames_0 = []
coords_1 = []
frames_1 = []


def start(ssh0: SSHClient, ssh1: SSHClient):
    global pid0, pid1, coords_0, frames_0, coords_1, frames_1

    coords_0 = []
    frames_0 = []
    coords_1 = []
    frames_1 = []

    if pid0 is None:
        scp0 = SCPClient(ssh0.get_transport())
        scp0.put(
            dir_path / "to_copter",
            recursive=True,
            remote_path=f"/home/{config.drone_username}/",
        )
        scp0.close()

    if pid1 is None and not config.is_one_drone:
        scp1 = SCPClient(ssh1.get_transport())
        scp1.put(
            dir_path / "to_copter",
            recursive=True,
            remote_path=f"/home/{config.drone_username}/",
        )
        scp1.close()

    if pid0 is None:
        _, stdout, stderr = ssh0.exec_command(
            '/bin/bash -ic "python3 to_copter/flight0.py &"',
            get_pty=True,
            timeout=10,
        )
        stdout_str = stdout.read().decode()
        pid0 = int(stdout_str.split()[1].strip())
        stderr_str = stderr.read().decode()
        print("STDOUT:", stdout_str, "STDERR:", stderr_str, "PID: ", pid0)

    if pid1 is None and not config.is_one_drone:
        _, stdout, stderr = ssh1.exec_command(
            '/bin/bash -ic "python3 to_copter/flight1.py &"',
            get_pty=True,
            timeout=10,
        )
        stdout_str = stdout.read().decode()
        pid1 = int(stdout_str.split()[1].strip())
        stderr_str = stderr.read().decode()
        print("STDOUT:", stdout_str, "STDERR:", stderr_str, "PID: ", pid1)


def perform_action(action: str):
    global pid0, pid1

    pid0 = None
    pid1 = None

    mav0 = mavutil.mavlink_connection(
        f"udpout:{config.drone_ip0}:{config.drone_mavlink_port0}",
    )
    if not config.is_one_drone:
        mav1 = mavutil.mavlink_connection(
            f"udpout:{config.drone_ip1}:{config.drone_mavlink_port1}"
        )

    msg = action.upper().encode("utf-8")

    mav0.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg)
    if not config.is_one_drone:
        mav1.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg)

    mav0.close()
    if not config.is_one_drone:
        mav1.close()


@app.post("/api/action", response_model=Message)
def handle_button_action(button_data: ButtonAction):
    if button_data.action == Action.start_mission:
        ssh0, ssh1 = SSHClient(), SSHClient()
        connect(ssh0, ssh1)
        start(ssh0, ssh1)
        disconnect(ssh0, ssh1)
        message = "[INFO] Полетное задание запущено"
    elif button_data.action == Action.emergency_shutdown:
        perform_action("INTERRUPT")
        message = "[ALERT] Экстренное выключение активировано"
    elif button_data.action == Action.land:
        perform_action("LAND")
        message = "[INFO] Посадка инициирована"
    elif button_data.action == Action.pause:
        perform_action("PAUSE")
        message = "[INFO] Полет приостановлен"
    else:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail="Unknown action"
        )

    return Message(message=message)


def check_error(stdout_str: str) -> bool:
    if "ERROR: ros.mavros.fcu" in stdout_str:
        return True
    if "RPi health: system throttled to prevent damage" in stdout_str:
        return True
    if "RPi health: your system is susceptible to throttling" in stdout_str:
        return True
    if (
        "RPi health: not enough power for onboard computer, flight inadvisable"
        in stdout_str
    ):
        return True
    if "RPi health: power supply cannot provide enough power" in stdout_str:
        return True
    return False


def get_battery(ssh0: SSHClient, ssh1: SSHClient, drone_id: int) -> Tuple[float, float]:
    if drone_id == 1 and config.is_one_drone:
        return 0, 0
    try:
        _, stdout, _ = (ssh0 if drone_id == 0 else ssh1).exec_command(
            'bash -ic "rostopic echo /mavros/battery -n 1 | grep -e "^voltage" -e "^percentage""',
            timeout=10,
            get_pty=True,
        )
        stdout_str = stdout.read().decode()
        print("STDOUT: ", stdout_str)
        voltage = float(stdout_str.split("\n")[0].split(":")[1].strip())
        percentage = float(stdout_str.split("\n")[1].split(":")[1].strip())
        return round(voltage, 2), round(percentage * 100, 2)
    except Exception as e:
        print("ERROR: ", e)
        return 0, 0


@app.get("/api/status/{drone_id}", response_model=Status)
def get_status(drone_id: int):
    ssh0, ssh1 = SSHClient(), SSHClient()
    connect(ssh0, ssh1)
    connection = True
    readiness = True
    try:
        _, stdout, _ = (ssh0 if drone_id == 0 else ssh1).exec_command(
            'bash -ic "rosrun clover selfcheck.py"',
            timeout=config.selfcheck_timeout,
            get_pty=True,
        )
        stdout_str = stdout.read().decode()
        if check_error(stdout_str):
            print("STDOUT: ", stdout_str)
            readiness = False
    except Exception as e:
        print("ERROR: ", e)
        connection = False
        readiness = False
    battery_voltage, battery_percent = get_battery(ssh0, ssh1, drone_id)
    disconnect(ssh0, ssh1)
    return Status(
        connection=connection,
        readiness=readiness,
        battery_voltage=battery_voltage,
        battery_percent=battery_percent,
    )


@app.post("/api/reset_pids/{drone_id}")
def reset_pids(drone_id: int):
    global pid0, pid1
    if drone_id == 0:
        pid0 = None
    elif drone_id == 1:
        pid1 = None


@app.get("/api/state/{drone_id}", response_model=DroneState)
def get_drone_state(drone_id: int):
    return drone_state_repo.get_state(drone_id)


@app.post("/api/state/{drone_id}", response_model=DroneState)
def update_drone_state(drone_id: int, new_state: DroneState):
    drone_state_repo.update_state(drone_id, new_state)
    return new_state


@app.websocket("/api/process-frame/{drone_id}")
async def websocket_process_frame(websocket: WebSocket, drone_id: int):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_bytes()
            if data == b"stop":
                final_img = None
                coordinates = None
            else:
                pil_image = Image.open(io.BytesIO(data))
                image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
                final_img, coordinates, _ = main_fire_detection(image)

            if drone_id == 0:
                frames_0.append(final_img)
            elif drone_id == 1:
                frames_1.append(final_img)

            await websocket.send_json({"coordinates": coordinates})
    except WebSocketDisconnect:
        print(f"Frame WebSocket disconnected for drone {drone_id}")


@app.websocket("/api/process-coords/{drone_id}")
async def websocket_process_coords(websocket: WebSocket, drone_id: int):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_json()
            if drone_id == 0:
                coords_0.extend(data["coords"])
            elif drone_id == 1:
                coords_1.extend(data["coords"])
    except WebSocketDisconnect:
        print(f"Coords WebSocket disconnected for drone {drone_id}")


@app.websocket("/api/fire-data/{drone_id}")
async def websocket_fire_data(websocket: WebSocket, drone_id: int):
    await websocket.accept()
    try:
        while True:
            await asyncio.sleep(0.2)

            current_frames = []

            if drone_id == 0 and frames_0:
                current_frames = frames_0.copy()
                frames_0.clear()

            if drone_id == 1 and frames_1:
                current_frames = frames_1.copy()
                frames_1.clear()

            if current_frames:
                fire_data = []
                for frame in current_frames:
                    if frame is None:
                        await websocket.close()
                        return
                    _, buffer = cv2.imencode(".png", frame)
                    image_data = base64.b64encode(buffer).decode("utf-8")
                    fire_data.append({"image_data": image_data})

                await websocket.send_json(fire_data)
    except WebSocketDisconnect:
        print(f"Fire data WebSocket disconnected for drone {drone_id}")
    except Exception as e:
        print(f"Error in /api/fire-data/{drone_id}:", e)


@app.get("/api/get-coords", response_model=List[Tuple[float, float, float]])
def get_coords():
    points = coords_0.copy() + coords_1.copy()
    print(points)
    return clusterize_coords(points, threshold=0.25)
