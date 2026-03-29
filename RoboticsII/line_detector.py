import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LidarDetector(Node):
    def __init__(self):
        super().__init__('lidar_detector')
        self.cones_range_cutoff = 10.0

        self.sub1 = self.create_subscription(PointCloud2, '/fsds/lidar/Lidar1', self.lidar_callback, 10)
        self.sub2 = self.create_subscription(PointCloud2, '/fsds/lidar/Lidar2', self.lidar_callback, 10)
        self.publisher = self.create_publisher(PoseArray, '/cones', 10)

    def lidar_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = type('Point', (), {'x': p[0], 'y': p[1], 'z': p[2]})()
            points.append(pt)

        if len(points) < 2:
            return

        cones = self.find_cones(points)

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = cones
        self.publisher.publish(pose_array)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def points_group_to_cone(self, points_group):
        pose = Pose()
        pose.position.x = sum(p.x for p in points_group) / len(points_group)
        pose.position.y = sum(p.y for p in points_group) / len(points_group)
        pose.position.z = 0.0
        return pose

    def find_cones(self, points):
        cones = []
        current_group = []
        i = 0
        previous_point = points[i]
        for i in range(1, len(points)):
            current_point = points[i]
            distance_to_last_point = self.distance(
                current_point.x, current_point.y,
                previous_point.x, previous_point.y
            )
            if distance_to_last_point < 0.1:
                current_group.append(current_point)
            else:
                if len(current_group) > 0:
                    cone = self.points_group_to_cone(current_group)
                    distance_to_cone = self.distance(0, 0, cone.position.x, cone.position.y)
                    if distance_to_cone < self.cones_range_cutoff:
                        cones.append(cone)
                    current_group = []
            previous_point = current_point
        return cones

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
