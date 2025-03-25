import math
from threading import Thread

import requests
import rospy
from clover import srv
from mavros_msgs.srv import CommandBool
from pymavlink import mavutil
from std_srvs.srv import Trigger

rospy.init_node("flight0")

get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
navigate = rospy.ServiceProxy("navigate", srv.Navigate)
navigate_global = rospy.ServiceProxy("navigate_global", srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy("set_altitude", srv.SetAltitude)
set_yaw = rospy.ServiceProxy("set_yaw", srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy("set_yaw_rate", srv.SetYawRate)
set_position = rospy.ServiceProxy("set_position", srv.SetPosition)
set_velocity = rospy.ServiceProxy("set_velocity", srv.SetVelocity)
set_attitude = rospy.ServiceProxy("set_attitude", srv.SetAttitude)
set_rates = rospy.ServiceProxy("set_rates", srv.SetRates)
land = rospy.ServiceProxy("land", Trigger)

arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

API_BASEURL = "http://192.168.2.164:8000/api"
# API_BASEURL = "http://192.168.0.30:8000/api"

LAND = False
INTERRUPT = False
PAUSE = False


def listen_for_status_text():
    """
    Listen for MAVLink STATUSTEXT messages and set corresponding
    global flags when specific commands are received.
    """
    global LAND, INTERRUPT, PAUSE
    connection_str = "udpin:0.0.0.0:14550"
    mav = mavutil.mavlink_connection(connection_str)
    rospy.loginfo("Listening for STATUSTEXT messages on port 14550...")

    while not rospy.is_shutdown():
        msg = mav.recv_match(type="STATUSTEXT", blocking=True)
        if msg is not None:
            text = (
                msg.text.decode("utf-8", errors="ignore")
                if isinstance(msg.text, bytes)
                else msg.text
            )
            rospy.loginfo("Received STATUSTEXT: {}".format(text))
            cmd = text.strip().upper()
            if cmd == "LAND":
                LAND = True
            elif cmd == "INTERRUPT":
                INTERRUPT = True
            elif cmd == "PAUSE":
                if PAUSE:
                    PAUSE = False
                else:
                    PAUSE = True


status_listener_thread = Thread(target=listen_for_status_text, daemon=True)
status_listener_thread.start()


def navigate_wait(
    x: float = 0,
    y: float = 0,
    z: float = 0,
    yaw: float = float("nan"),
    speed: float = 0.3,
    frame_id: str = "",
    auto_arm: bool = False,
    tolerance: float = 0.2,
) -> None:
    if LAND or INTERRUPT or PAUSE:
        return

    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        if LAND:
            set_velocity(vx=0, vy=0, vz=0, frame_id="aruco_map")
            land_wait()
            exit()
        if INTERRUPT:
            set_velocity(vx=0, vy=0, vz=0, frame_id="aruco_map")
            arming(False)
            exit()
        if PAUSE:
            telem = get_telemetry(frame_id="aruco_map")
            while not rospy.is_shutdown() and PAUSE:
                if LAND:
                    land_wait()
                    exit()
                if INTERRUPT:
                    arming(False)
                    exit()
                set_position(x=telem.x, y=telem.y, z=telem.z, frame_id="aruco_map")
                rospy.sleep(0.2)
            navigate(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                speed=speed,
                frame_id=frame_id,
                auto_arm=auto_arm,
            )

        telem = get_telemetry(frame_id="navigate_target")
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
            break

        rospy.sleep(0.2)


def land_wait():
    if INTERRUPT:
        return
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


# 4->14->13->3->4
navigate_wait(z=1.5, frame_id="body", auto_arm=True)
navigate_wait(z=1.5, frame_id="aruco_4")
navigate_wait(z=1.5, frame_id="aruco_14")
navigate_wait(z=1.5, frame_id="aruco_13")
navigate_wait(z=1.5, frame_id="aruco_3")
navigate_wait(z=1.5, frame_id="aruco_4")

try:
    rospy.loginfo("Setting state to ready to land")
    requests.post(f"{API_BASEURL}/state/1", json={"ready_to_land": True})
except Exception as e:
    rospy.loginfo(f"Error setting state: {e}")

cnt_errors = 0
rospy.loginfo("Waiting for drone 0 to be ready to land")
while not rospy.is_shutdown():
    if cnt_errors > 3:
        rospy.loginfo("Drone 0 is not ready to land, breaking")
        break
    try:
        r = requests.get(f"{API_BASEURL}/state/0", timeout=3)
        r.raise_for_status()
        if r.json()["ready_to_land"]:
            break
    except Exception as e:
        rospy.loginfo(f"Error getting state: {e}")
        cnt_errors += 1
    rospy.sleep(0.2)

land_wait()

try:
    requests.post(f"{API_BASEURL}/state/1", json={"ready_to_land": False})
except Exception as e:
    rospy.loginfo(f"Error setting final state: {e}")
