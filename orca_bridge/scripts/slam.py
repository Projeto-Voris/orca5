import math

import orbslam3_msgs.msg
import numpy as np
import sensor_msgs.msg
from sensor_msgs_py import point_cloud2
import geometry_msgs.msg

import geometry
import sub


def scale_cloud(msg: sensor_msgs.msg.PointCloud2, scale: float):
    """Scale a point cloud."""

    cloud = point_cloud2.read_points_numpy(msg)
    #multiplica a nuvem de pontos pelo fator de escala
    cloud *= scale
    return point_cloud2.create_cloud_xyz32(msg.header, cloud)

def rf_distance(msg_pose: geometry_msgs.msg.PoseStamped, msg_cloud: sensor_msgs.msg.PointCloud2) -> float:
        """Emulate a sonar rangefinder."""
        # msg é pose e pointclound2

        # Defini parametros do sonar
        # Ângulo do feixe
        half_beam_angle_d = 20.0  # Beam angle of 40d
        # Threshold para verificar se o ponto está dentro do cone do sonar
        max_tan_theta_sq = math.tan(math.radians(half_beam_angle_d)) ** 2
        min_points = 15

        # Get the pose of the world in the camera frame
        t_camera_world = geometry.Pose.from_pose_msg(msg_pose).inverse()

        z_values = []
        for p in point_cloud2.read_points(msg_cloud, field_names=["x", "y", "z"], skip_nans=True):
            # Move the tracked points from the world frame to the camera frame
            point_f_world = geometry.Pose()
            point_f_world.set_position(*p)
            point_f_camera = t_camera_world.mult(point_f_world)

            # Reject z values that are outside the beam
            max_xy_dist_sq = point_f_camera.p[2] ** 2 * max_tan_theta_sq
            xy_dist_sq = point_f_camera.p[0] ** 2 + point_f_camera.p[1] ** 2
            if xy_dist_sq <= max_xy_dist_sq:
                z_values.append(point_f_camera.p[2])

        if len(z_values) < min_points:
            # print(f'Too few points inside the sonar beam: {len(z_values)} < {min_points}')
            return 0

        # The median is robust to outliers
        return float(np.percentile(np.array(z_values), 50.0))


class LowPassFilter:
    """A simple low-pass filter implementation using exponential moving average."""

    def __init__(self, alpha: float):
        """
        Initialize the low-pass filter.

        Args:
            alpha: Smoothing factor between 0 and 1.
                   Higher values (closer to 1) = less smoothing, faster response.
                   Lower values (closer to 0) = more smoothing, slower response.
                   Typical values: 0.1 to 0.5
        """
        if not 0 < alpha <= 1:
            raise ValueError("Alpha must be between 0 (exclusive) and 1 (inclusive)")

        self.alpha = alpha
        self.value = None

    def update(self, measurement: float) -> float:
        if self.value is None:
            # Initialize with the first measurement
            self.value = measurement
        else:
            # Apply exponential moving average: y[n] = α * x[n] + (1 - α) * y[n-1]
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value

        return self.value

    def get_value(self) -> float | None:
        return self.value

    def reset(self):
        self.value = None


class SlamMap:
    """A single SLAM map"""

    def __init__(self, map_id: int, sonar_rf: float, slam_rf: float, t_map_base: geometry.Pose):
        # The map id from ORB_SLAM3
        self.map_id = map_id

        # Rangefinder readings for logging
        self.sonar_rf = float(sonar_rf)
        self.slam_rf = float(slam_rf)

        # Scale factor for this map. Default to 1.0 until we get a good reading, then use a low pass filter
        self.scale = 1.0
        self.scale_filter = LowPassFilter(0.1)
        self.update_scale(sonar_rf, slam_rf)

        # Initialize map -> slam from map -> base
        self.t_map_slam = t_map_base

        # The latest pose of the base link in the slam frame
        self.t_slam_base = None

    def update_scale(self, sonar_rf: float, slam_rf: float):
        self.sonar_rf = float(sonar_rf)
        self.slam_rf = float(slam_rf)

        # Drop bad readings
        if slam_rf > 0.05:
            self.scale = float(self.scale_filter.update(sonar_rf / slam_rf))

    def update_pose(self, t_slam_base: geometry.Pose):
        self.t_slam_base = t_slam_base


class SlamMaps:
    """A set of SLAM maps."""

    def __init__(self):
        # The set of maps that we've seen
        self.maps: dict[int, SlamMap] = {}

        # The current map
        self.current_map: SlamMap | None = None

    def update(self, msg_status: orbslam3_msgs.msg.SlamStatus, msg_pose: geometry_msgs.msg.PoseStamped, msg_cloud: sensor_msgs.msg.PointCloud2, sub: sub.Sub, logger):
        # Only trust the map data if we are tracking
        if msg_status.tracking_state != orbslam3_msgs.msg.SlamStatus.TRACKING_OK:
            logger.error('Not tracking, do not update')
            return

        slam_rf_distance = rf_distance(msg_pose, msg_cloud)

        if self.current_map is None or msg_status.map_id not in self.maps:
            logger.info(f'Create map {msg_status.map_id}')
            t_map_base = sub.t_map_base_ned.ned_to_enu_frame()  # Use axes-only conversion
            self.maps[msg_status.map_id] = SlamMap(msg_status.map_id, sub.sonar_rf_distance, slam_rf_distance, t_map_base)
            self.current_map = self.maps[msg_status.map_id]
        else:
            if self.current_map.map_id != msg_status.map_id:
                # This never happens: ORB_SLAM3 creates a new map and then merges in the old map(s).
                # Sadly, we lose the old pose and scale information.
                logger.error(f'Switch to map {msg_status.map_id} -- WTF')
                self.current_map = self.maps[msg_status.map_id]
            self.current_map.update_scale(sub.sonar_rf_distance, slam_rf_distance)
