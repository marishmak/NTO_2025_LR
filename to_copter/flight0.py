import math

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node("flight")

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


def navigate_wait(
    x=0,
    y=0,
    z=0,
    yaw=float("nan"),
    speed=0.5,
    frame_id="",
    auto_arm=False,
    tolerance=0.2,
):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id="navigate_target")
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
            break
        rospy.sleep(0.2)


def land_wait():
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
