from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from paramiko import SSHClient
from pymavlink import mavutil
from scp import SCPClient

from config import config
from schemas import Action, ButtonAction, Message, Status

dir_path = Path(__file__).parent.parent


def connect(ssh0: SSHClient, ssh1: SSHClient):
    ssh0.load_system_host_keys()
    ssh0.connect(
        config.drone_ip0,
        config.drone_port0,
        config.drone_username,
        config.drone_password,
    )
    if config.drone_ip1:
        ssh1.load_system_host_keys()
        ssh1.connect(
            config.drone_ip1,
            config.drone_port1,
            config.drone_username,
            config.drone_password,
        )


def disconnect(ssh0: SSHClient, ssh1: SSHClient):
    ssh0.close()
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


def start(ssh0: SSHClient, ssh1: SSHClient):
    global pid0, pid1

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
    disconnect(ssh0, ssh1)
    return Status(connection=connection, readiness=readiness)


@app.post("/api/reset_pids")
def reset_pids():
    global pid0, pid1
    pid0 = None
    pid1 = None
