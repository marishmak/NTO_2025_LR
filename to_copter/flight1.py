import json
import math
import sys
from threading import Thread
from typing import Tuple

import cv2
import numpy as np
import requests
import rospy
from clover import long_callback, srv
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge
from mavros_msgs.srv import CommandBool
from pymavlink import mavutil
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped
from tf2_ros import Buffer, TransformListener
from websockets.sync.client import connect as websocket_connect

# sys.stdout = open("flight1.log", "w")

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
set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)


API_BASEURL = "http://192.168.0.36:8000/api"  # test flight
# API_BASEURL = "http://192.168.2.164:8000/api"
# API_BASEURL = "http://192.168.0.30:8000/api"  # final flight
# API_BASEURL = "http://127.0.0.1:8000/api"

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
object_point_60_60 = np.array(
    [
        (-0.6 / 2, -0.6 / 2, 0),  # Bottom-left corner
        (0.6 / 2, -0.6 / 2, 0),  # Bottom-right corner
        (0.6 / 2, 0.6 / 2, 0),  # Top-right corner
        (-0.6 / 2, 0.6 / 2, 0),  # Top-left corner
    ]
)
object_point_60_90 = np.array(
    [
        (-0.6 / 2, -0.9 / 2, 0),  # Bottom-left corner
        (0.6 / 2, -0.9 / 2, 0),  # Bottom-right corner
        (0.6 / 2, 0.9 / 2, 0),  # Top-right corner
        (-0.6 / 2, 0.9 / 2, 0),  # Top-left corner
    ]
)
object_point_90_90 = np.array(
    [
        (-0.9 / 2, -0.9 / 2, 0),  # Bottom-left corner
        (0.9 / 2, -0.9 / 2, 0),  # Bottom-right corner
        (0.9 / 2, 0.9 / 2, 0),  # Top-right corner
        (-0.9 / 2, 0.9 / 2, 0),  # Top-left corner
    ]
)
# other things
counter = 0
do_recognition = False
websocket_frame = websocket_connect(
    f"{API_BASEURL.replace('http', 'ws')}/process-frame/1"
)
websocket_coords = websocket_connect(
    f"{API_BASEURL.replace('http', 'ws')}/process-coords/1"
)


def transform_xyz_yaw(
    x: float, y: float, z: float, frame_from: str, frame_to: str, curr_time: rospy.Time
) -> Tuple[float, float, float]:
    point_from = PointStamped()
    point_from.header.frame_id = frame_from
    point_from.header.stamp = curr_time
    point_from.point.x = x
    point_from.point.y = y
    point_from.point.z = z
    point_to = tf_buffer.transform(point_from, frame_to, rospy.Duration(0.2))
    return point_to.point.x, point_to.point.y, point_to.point.z


def polygon_area(points):
    n = len(points)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += points[i][0] * points[j][1]
        area -= points[j][0] * points[i][1]
    return abs(area) / 2.0


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
        curr_time = rospy.get_rostime()

        _, img_encoded = cv2.imencode(".jpg", image)
        websocket_frame.send(img_encoded.tobytes())
        resp = websocket_frame.recv()
        coords = json.loads(resp)["coordinates"]
        rospy.loginfo(f"Coords: {coords}")

        corners_points = []
        for coord in coords:
            image_point = [
                (coord[0], coord[1] + coord[3]),
                (coord[0] + coord[2], coord[1] + coord[3]),
                (coord[0] + coord[2], coord[1]),
                (coord[0], coord[1]),
            ]
            area = polygon_area(image_point)
            print("Area:", area)

            if area <= 2000:
                object_point = object_point_60_60
            elif 2000 < area <= 3000:
                object_point = object_point_60_90
            else:
                object_point = object_point_90_90

            _, rvec, tvec = cv2.solvePnP(
                np.array(object_point, dtype="float32"),
                np.array(image_point, dtype="float32"),
                camera_matrix,
                dist_coeffs,
            )

            R, _ = cv2.Rodrigues(rvec)

            corner_points = []
            for point in object_point:
                point_cam = (R.dot(point.reshape(3, 1))).flatten() + tvec.flatten()
                world_pt = transform_xyz_yaw(
                    point_cam[0],
                    point_cam[1],
                    point_cam[2],
                    "main_camera_optical",
                    "aruco_map",
                    curr_time,
                )
                corner_points.append(world_pt)

            corners_points.append(
                [
                    sum(p[0] for p in corner_points)
                    / 4,  # X CENTER (average of all x coordinates)
                    sum(p[1] for p in corner_points)
                    / 4,  # Y CENTER (average of all y coordinates)
                    polygon_area(corner_points) * 10000,
                ]
            )

        rospy.loginfo(f"Corners points: {corners_points}")

        websocket_coords.send(json.dumps({"coords": corners_points}))
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
        f"{API_BASEURL}/state/1",
        json={"ready_to_start": True, "ready_to_land": False},
        timeout=10,
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


# flight
set_effect(r=255, g=165, b=0)
navigate_wait(z=1.5, frame_id="body", auto_arm=True)
set_effect(r=255, g=165, b=0)

cnt_errors = 0
rospy.loginfo("Waiting for drone 0 to finish scanning")
while not rospy.is_shutdown():
    if cnt_errors > 3:
        rospy.loginfo("Errors, drone 0 is not finished scanning, breaking")
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

response = requests.get(f"{API_BASEURL}/get-coords", timeout=10)
coords = response.json()
coords = [(coord[0], coord[1]) for coord in coords]
coords.sort(key=lambda x: x[1])

for coord in coords:
    if coord[0] <= 0.9 and coord[1] <= 0.9:
        rospy.loginfo("Coord is too close to first, skipping")
        continue
    navigate_wait(x=coord[0], y=coord[1], z=1.5, frame_id="aruco_map")
    rospy.sleep(0.2)
    set_effect(effect="blink", r=0, g=0, b=255)
    navigate_wait(x=coord[0], y=coord[1], z=1.3, frame_id="aruco_map")
    rospy.sleep(0.2)
    navigate_wait(x=coord[0], y=coord[1], z=1.5, frame_id="aruco_map")
    rospy.sleep(0.2)
    set_effect(r=255, g=165, b=0)

navigate_wait(z=1.5, frame_id="aruco_4")

# SYNC LANDING
try:
    rospy.loginfo("Setting state to ready to land")
    requests.post(
        f"{API_BASEURL}/state/1",
        json={"ready_to_start": False, "ready_to_land": True},
        timeout=10,
    )
except Exception as e:
    rospy.loginfo(f"Error setting state: {e}")

land_wait()

set_effect(r=0, g=0, b=0)

try:
    requests.post(
        f"{API_BASEURL}/state/1",
        json={"ready_to_start": False, "ready_to_land": False},
        timeout=10,
    )
    requests.post(f"{API_BASEURL}/reset_pids/1", timeout=10)
except Exception as e:
    rospy.loginfo(f"Error setting final state: {e}")

websocket_frame.close()
websocket_coords.close()
