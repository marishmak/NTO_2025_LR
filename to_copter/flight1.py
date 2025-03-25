import math
import sys
from io import BytesIO
from threading import Thread
from typing import Tuple

import cv2
import numpy as np
import requests
import rospy
from clover import long_callback, srv
from cv_bridge import CvBridge
from mavros_msgs.srv import CommandBool
from pymavlink import mavutil
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped
from tf2_ros import Buffer, TransformListener

sys.stdout = open("flight1.log", "w")

rospy.init_node("flight1")

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


# API_BASEURL = "http://192.168.0.36:8000/api"  # test flight
# API_BASEURL = "http://192.168.2.164:8000/api"
# API_BASEURL = "http://192.168.0.30:8000/api"  # final flight
API_BASEURL = "http://127.0.0.1:8000/api"

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


bridge = CvBridge()
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer)
# read camera info
camera_info = rospy.wait_for_message("main_camera/camera_info", CameraInfo)
camera_matrix = np.reshape(np.array(camera_info.K, dtype="float64"), (3, 3))
dist_coeffs = np.array(camera_info.D, dtype="float64")
object_point = np.array(
    [
        (-90 / 2, -60 / 2, 0),  # Bottom-left corner  (-45, -30, 0)
        (90 / 2, -60 / 2, 0),  # Bottom-right corner (45, -30, 0)
        (90 / 2, 60 / 2, 0),  # Top-right corner    (45, 30, 0)
        (-90 / 2, 60 / 2, 0),  # Top-left corner     (-45, 30, 0)
    ]
)
counter = 0
do_recognition = False


def transform_xyz_yaw(
    x: float, y: float, z: float, frame_from: str, frame_to: str
) -> Tuple[float, float, float]:
    point_from = PointStamped()
    point_from.header.frame_id = frame_from
    point_from.header.stamp = rospy.get_rostime()
    point_from.point.x = x
    point_from.point.y = y
    point_from.point.z = z
    point_to = tf_buffer.transform(point_from, frame_to, rospy.Duration(0.2))
    return point_to.point.x, point_to.point.y, point_to.point.z


@long_callback
def img_callback(msg):
    global counter, do_recognition

    if not do_recognition:
        return

    counter += 1
    if counter % 5 != 0:
        return

    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        _, img_encoded = cv2.imencode(".jpg", image)
        resp = requests.post(
            f"{API_BASEURL}/process-frame",
            files={"file": ("image.jpg", BytesIO(img_encoded), "image/jpeg")},
            timeout=10,
        )
        if resp.status_code != 200:
            rospy.loginfo(f"Error processing frame: {resp.status_code}")
            return
        coords = resp.json()
        rospy.loginfo(f"Coords: {coords}")

        corners_points = []
        for coord in coords:
            _, rvec, tvec = cv2.solvePnP(
                np.array(object_point, dtype="float32"),
                np.array(
                    [
                        (coord[0], coord[1] + coord[3]),
                        (coord[0] + coord[2], coord[1] + coord[3]),
                        (coord[0] + coord[2], coord[1]),
                        (coord[0], coord[1]),
                    ],
                    dtype="float32",
                ),
                camera_matrix,
                dist_coeffs,
            )

            corner_points = []
            for point in object_point:
                point_3d = np.array([[point]], dtype="float32")
                point_2d, _ = cv2.projectPoints(
                    point_3d, rvec, tvec, camera_matrix, dist_coeffs
                )

                x_map, y_map, z_map = transform_xyz_yaw(
                    float(point_2d[0][0][0]),  # X
                    float(point_2d[0][0][1]),  # Y
                    float(tvec[2][0]),  # Z (depth)
                    "main_camera_optical",
                    "aruco_map",
                )
                corner_points.append((x_map, y_map, z_map))

            corners_points.append(
                [
                    sum(p[0] for p in corner_points)
                    / 4,  # X CENTER (average of all x coordinates)
                    sum(p[1] for p in corner_points)
                    / 4,  # Y CENTER (average of all y coordinates)
                    abs(
                        (corner_points[1][0] - corner_points[0][0])
                        * 100
                        * (corner_points[2][1] - corner_points[1][1])
                        * 100
                    ),  # AREA (width * height)
                ]
            )

        rospy.loginfo(f"Corners points: {corners_points}")

        requests.post(
            f"{API_BASEURL}/process-coords",
            json={"coords": corners_points},
            timeout=10,
        )
    except Exception as e:
        rospy.loginfo(f"Error processing frame: {e}")


image_sub = rospy.Subscriber(
    "main_camera/image_raw_throttled",
    Image,
    img_callback,
    queue_size=10,
    buff_size=320 * 240 * 3 * 50,
)


def navigate_wait(
    x: float = 0,
    y: float = 0,
    z: float = 0,
    yaw: float = float("nan"),
    speed: float = 0.2,
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


# SYNC STARTING
try:
    rospy.loginfo("Setting state to ready to start")
    requests.post(
        f"{API_BASEURL}/state/1", json={"ready_to_start": True, "ready_to_land": False}
    )
except Exception as e:
    rospy.loginfo(f"Error setting state: {e}")

cnt_errors = 0
rospy.loginfo("Waiting for drone 0 to be ready to start")
while not rospy.is_shutdown():
    if cnt_errors > 3:
        rospy.loginfo("Drone 0 is not ready to start, breaking")
        break
    try:
        r = requests.get(f"{API_BASEURL}/state/0", timeout=3)
        r.raise_for_status()
        if r.json()["ready_to_start"]:
            break
    except Exception as e:
        rospy.loginfo(f"Error getting state: {e}")
        cnt_errors += 1
    rospy.sleep(0.2)


# 4->34->33->3->4
navigate_wait(z=1.5, frame_id="body", auto_arm=True)
do_recognition = True
navigate_wait(z=1.5, frame_id="aruco_4")
navigate_wait(z=1.5, frame_id="aruco_34")
navigate_wait(z=1.5, frame_id="aruco_33")
navigate_wait(z=1.5, frame_id="aruco_3")
navigate_wait(z=1.5, frame_id="aruco_4")
do_recognition = False

# SYNC LANDING
try:
    rospy.loginfo("Setting state to ready to land")
    requests.post(
        f"{API_BASEURL}/state/1", json={"ready_to_start": False, "ready_to_land": True}
    )
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
    requests.post(
        f"{API_BASEURL}/state/1", json={"ready_to_start": False, "ready_to_land": False}
    )
    requests.post(f"{API_BASEURL}/reset_pids/1", timeout=10)
except Exception as e:
    rospy.loginfo(f"Error setting final state: {e}")
