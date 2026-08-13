import cv2
import math
import numpy as np


def image_from_msg(msg, width, height):
    if hasattr(msg, "format"):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    else:
        from cv_bridge import CvBridge
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    if image is None:
        raise ValueError("Failed to decode camera message")
    return cv2.cvtColor(cv2.resize(image, (width, height)), cv2.COLOR_BGR2RGB).transpose(2, 0, 1)


def ego_vector(msg):
    linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
    speed = (linear.x ** 2 + linear.y ** 2 + linear.z ** 2) ** 0.5
    return np.asarray([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z, speed], np.float32)


def ego_localization_vector(
    odom, imu, vehicle_status, gps, stamp, route=None, velocity_scale=1.0
):
    """V2 normalized 13-D Vehicle Status + IMU + GPS-health vector."""
    velocity = vehicle_status.velocity
    acceleration = vehicle_status.acceleration
    vx = float(velocity.x) * float(velocity_scale)
    vy = float(velocity.y) * float(velocity_scale)
    speed = math.hypot(vx, vy)
    yaw_rate = float(odom.twist.twist.angular.z)
    # beta_drive morai_msgs uses `steer`; retain wheel_angle compatibility for
    # previously recorded bags with the older local message definition.
    steering_deg = getattr(
        vehicle_status, "steer", getattr(vehicle_status, "wheel_angle", 0.0)
    )
    steering_rad = math.radians(float(steering_deg))

    gps_stamp = getattr(getattr(gps, "header", None), "stamp", None)
    gps_time = gps_stamp.to_sec() if gps_stamp is not None else 0.0
    gps_age = max(0.0, float(stamp) - gps_time) if gps_time > 0.0 else 99.0
    gps_status = getattr(getattr(gps, "status", None), "status", -1)
    latitude = float(getattr(gps, "latitude", float("nan")))
    longitude = float(getattr(gps, "longitude", float("nan")))
    gps_valid = float(gps_status >= 0 and np.isfinite(latitude) and np.isfinite(longitude))

    covariance = np.asarray(odom.pose.covariance, dtype=np.float32)
    position_sigma = math.sqrt(max(float(covariance[0] + covariance[7]), 0.0))
    localization_confidence = math.exp(-position_sigma / 5.0)

    route_confidence = 0.0
    if route is not None and getattr(route, "poses", None):
        distance = min(
            math.hypot(p.pose.position.x, p.pose.position.y) for p in route.poses
        )
        route_confidence = math.exp(-distance / 5.0)

    return np.asarray([
        speed / 20.0,
        vx / 20.0,
        vy / 20.0,
        yaw_rate / 2.0,
        steering_rad / 0.7,
        float(acceleration.x) / 10.0,
        float(imu.angular_velocity.z) / 2.0,
        float(imu.linear_acceleration.x) / 10.0,
        float(imu.linear_acceleration.y) / 10.0,
        gps_valid,
        min(gps_age / 5.0, 1.0),
        localization_confidence,
        route_confidence,
    ], dtype=np.float32)
