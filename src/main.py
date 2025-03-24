from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from paramiko import SSHClient
from scp import SCPClient

from config import config
from schemas import Action, ButtonAction, Message, Status

dir_path = Path(__file__).parent.parent


def connect(ssh0, ssh1):
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


def disconnect(ssh0, ssh1):
    ssh0.close()
    ssh1.close()


@asynccontextmanager
async def lifespan(app: FastAPI):
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


def start(ssh0, ssh1):
    global pid0, pid1

    if pid0 is None:
        scp0 = SCPClient(ssh0.get_transport())
        scp0.put(
            dir_path / "to_copter",
            recursive=True,
            remote_path=f"/home/{config.drone_username}/",
        )
        scp0.close()

    if pid1 is None and config.drone_ip1:
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
        # if stderr_str:
        #     raise HTTPException(
        #         status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        #         detail=stderr_str,
        #     )

    if pid1 is None and config.drone_ip1:
        _, stdout, stderr = ssh1.exec_command(
            '/bin/bash -ic "python3 to_copter/flight1.py &"',
            get_pty=True,
            timeout=10,
        )
        stdout_str = stdout.read().decode()
        pid1 = int(stdout_str.split()[1].strip())
        stderr_str = stderr.read().decode()
        print("STDOUT:", stdout_str, "STDERR:", stderr_str, "PID: ", pid1)
        # if stderr_str:
        #     raise HTTPException(
        #         status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        #         detail=stderr_str,
        #     )


def emergency_shutdown(ssh0, ssh1):
    global pid0, pid1

    if pid0 is not None:
        _, stdout, stderr = ssh0.exec_command(
            f'/bin/bash -ic "kill -9 {pid0}"',
            get_pty=True,
            timeout=10,
        )
        pid0 = None

    _, stdout, stderr = ssh0.exec_command(
        '/bin/bash -ic "python3 to_copter/interrupt.py"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )

    if not config.drone_ip1:
        return

    if pid1 is not None:
        _, stdout, stderr = ssh1.exec_command(
            f'/bin/bash -ic "kill -9 {pid1}"',
            get_pty=True,
            timeout=10,
        )
        pid1 = None

    _, stdout, stderr = ssh1.exec_command(
        '/bin/bash -ic "python3 to_copter/interrupt.py"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )


def land(ssh0, ssh1):
    global pid0, pid1

    if pid0 is not None:
        _, stdout, stderr = ssh0.exec_command(
            f'/bin/bash -ic "kill -9 {pid0}"',
            get_pty=True,
        )
        pid0 = None

    _, stdout, stderr = ssh0.exec_command(
        '/bin/bash -ic "python3 to_copter/land.py &"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )

    if not config.drone_ip1:
        return

    if pid1 is not None:
        _, stdout, stderr = ssh1.exec_command(
            f'/bin/bash -ic "kill -9 {pid1}"',
            get_pty=True,
        )
        pid1 = None

    _, stdout, stderr = ssh1.exec_command(
        '/bin/bash -ic "python3 to_copter/land.py &"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )


def pause(ssh0, ssh1):
    global pid0, pid1

    if pid0 is not None:
        _, stdout, stderr = ssh0.exec_command(
            f'/bin/bash -ic "kill -9 {pid0}"',
            get_pty=True,
            timeout=10,
        )
        pid0 = None
    _, stdout, stderr = ssh0.exec_command(
        '/bin/bash -ic "python3 to_copter/pause.py"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )

    if not config.drone_ip1:
        return

    if pid1 is not None:
        _, stdout, stderr = ssh1.exec_command(
            f'/bin/bash -ic "kill -9 {pid1}"',
            get_pty=True,
            timeout=10,
        )
        pid1 = None

    _, stdout, stderr = ssh1.exec_command(
        '/bin/bash -ic "python3 to_copter/pause.py"',
        get_pty=True,
        timeout=10,
    )
    print("STDOUT:", stdout.read().decode())
    stderr_str = stderr.read().decode()
    if stderr_str:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=stderr_str,
        )


@app.post("/api/action", response_model=Message)
def handle_button_action(button_data: ButtonAction):
    ssh0, ssh1 = SSHClient(), SSHClient()
    connect(ssh0, ssh1)
    if button_data.action == Action.start_mission:
        start(ssh0, ssh1)
        message = "[INFO] Полетное задание запущено"
    elif button_data.action == Action.emergency_shutdown:
        emergency_shutdown(ssh0, ssh1)
        message = "[ALERT] Экстренное выключение активировано"
    elif button_data.action == Action.land:
        land(ssh0, ssh1)
        message = "[INFO] Посадка инициирована"
    elif button_data.action == Action.pause:
        pause(ssh0, ssh1)
        message = "[INFO] Полет приостановлен"
    else:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail="Unknown action"
        )
    disconnect(ssh0, ssh1)

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

    if drone_id == 0:
        try:
            _, stdout, _ = ssh0.exec_command(
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
    elif drone_id == 1:
        try:
            _, stdout, _ = ssh1.exec_command(
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
