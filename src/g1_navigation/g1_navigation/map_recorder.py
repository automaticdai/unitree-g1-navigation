from datetime import datetime, timezone
from pathlib import Path
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from .map_io import VoxelMap, save_bundle
from .ros_helpers import parameter, xyz, spin


class MapRecorder(Node):
    def __init__(self):
        super().__init__('map_recorder')
        self.destination = parameter(self, 'session_directory', '')
        if not self.destination or not Path(self.destination).is_absolute():
            raise ValueError('session_directory must be an absolute new map bundle path')
        self.config_file = parameter(self, 'robot_config', '')
        self.fastlio_config = parameter(self, 'fastlio_config', '')
        self.grid_resolution = parameter(self, 'grid_resolution', 0.1)
        self.map = VoxelMap(parameter(self, 'voxel_resolution', 0.08),
                            parameter(self, 'max_voxels', 2_000_000))
        self.started = datetime.now(timezone.utc).isoformat()
        self.last = None
        self.error = ''
        self.create_subscription(PointCloud2, '/mapping/points', self.record, qos_profile_sensor_data)
        self.create_service(Trigger, '/mapping/save', self.save)

    def record(self, msg):
        try:
            if msg.header.frame_id != 'odom':
                raise ValueError('Expected mapping points in odom')
            self.map.add(xyz(msg))
            self.last = time.monotonic()
        except ValueError as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)

    def save(self, request, response):
        try:
            if self.last is None or time.monotonic() - self.last > 2.0:
                raise ValueError('No recent mapping data; save while the sensor stack is running')
            if self.error:
                raise ValueError(self.error)
            configs = {name: Path(path).read_text() for name, path in (
                ('robot', self.config_file), ('fastlio', self.fastlio_config)) if path}
            result = save_bundle(self.destination, self.map.points(), dict(
                schema_version=1, robot='G1-EDU-29', started=self.started,
                saved=datetime.now(timezone.utc).isoformat(),
                frame='map', source_frame='odom', floor_z=0.0,
                loop_closed=False, configs=configs,
                occupancy='Only observed ground is free; unseen cells remain unknown'),
                resolution=self.grid_resolution)
            response.success, response.message = True, str(result)
        except (ValueError, OSError, TypeError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main():
    spin(MapRecorder)
