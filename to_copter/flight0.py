import math
from threading import Thread

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
            set_velocity(vx=0, vy=0, vz=0, frame_id="body")
            land_wait()
            exit()
        if INTERRUPT:
            set_velocity(vx=0, vy=0, vz=0, frame_id="body")
            arming(False)
            exit()
        if PAUSE:
            set_velocity(vx=0, vy=0, vz=0, frame_id="body")
            while not rospy.is_shutdown() and PAUSE:
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


#  0->25->26->1->0
navigate_wait(z=1.5, frame_id="body", auto_arm=True)
navigate_wait(z=1.5, frame_id="aruco_0")
navigate_wait(z=1.5, frame_id="aruco_25")
navigate_wait(z=1.5, frame_id="aruco_26")
navigate_wait(z=1.5, frame_id="aruco_1")
navigate_wait(z=1.5, frame_id="aruco_0")

land_wait()
