#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, ParamSet
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
import math

# Follow waypoints without using yaw control, so the robot don't rotate to face the direction of movement

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.armed = False
        self.mode = ""
        self.connected = False

        self.target = None
        self.vel = None
        self.current_pose = None
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoints) # 20HZ para FCU conseguir ler
        self.velocity_timer = self.create_timer(0.05, self.publish_velocity) # 20HZ para FCU conseguir ler

        self.SERVICE_ARM = "/mavros/cmd/arming"
        self.SERVICE_SET_MODE = "/mavros/set_mode"
        self.TOPIC_STATE = "/mavros/state"
        self.SERVICE_SET_PARAM = "/mavros/param/set"
        self.SERVICE_GET_PARAM = "/mavros/param/get"
        self.TOPIC_SET_POSE_GLOBAL = "/mavros/setpoint_position/local"
        
        self.mavros_arm_cli = self.create_client(CommandBool, self.SERVICE_ARM)
        self.mavros_set_mode_cli = self.create_client(SetMode, self.SERVICE_SET_MODE)
        self.mavros_state_sub = self.create_subscription(State, self.TOPIC_STATE, self.state_callback, 10)
        self.mavros_set_param_cli = self.create_client(ParamSet, self.SERVICE_SET_PARAM)
        self.mavros_get_param_cli = self.create_client(ParamGet, self.SERVICE_GET_PARAM)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, "/model/bluerov2/odometry", self.odom_callback, qos)

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.twist_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(1.0, self.follow_wp)

        self.waypoints = [
            (0.0, 0.0, -2.0),
            (4.0, 0.0, -2.0),
            (4.0, 2.5, -2.0),
            (0.0, 2.5, -2.0),
            (0.0, 0.0, 0.0)
        ]
        self.current_wp = 0

    def generate_pathcurve(self, r, num_points):
        points = []
        for i in range(num_points):
            theta = 2*math.pi*i/num_points
            x = r*math.cos(theta)
            y = r*math.sin(theta)
            z = self.depth
            points.append((x, y, z))
        return points
    
    def distance(self, p1, p2):
        distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)
        return distance

    def arm(self, status: bool):
        request = CommandBool.Request()
        request.value = status
        future = self.mavros_arm_cli.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info('Service called successfully.')
            else:
                self.get_logger().error('Failed to call service.')
        except Exception as e:
            self.get_logger().error(f'Error calling service: {e}')
    
    def change_mode(self, mode: str):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mavros_set_mode_cli.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def move(self, x: float, y: float, z: float, u: float , v: float, w: float):
        self.target = (x, y, z)
        self.vel = (u, v, w)
    
    def publish_setpoints(self):
        if self.target is None:
            self.get_logger().warn('Target not set.')
            return
        
        x, y, z = self.target

        data = PoseStamped()
        data.header.stamp =self.get_clock().now().to_msg()
        data.header.frame_id = 'map'

        data.pose.position.x = float(x)
        data.pose.position.y = float(y)
        data.pose.position.z = float(z)

        data.pose.orientation.w = 1.0
        
        self.pos_pub.publish(data)

    def publish_velocity(self):
        if self.vel is None:
            self.get_logger().warn('Velocity not set.')
            return
        
        u, v, w = self.vel

        msg = Twist()

        msg.linear.x = float(u)
        msg.linear.y = float(v)
        msg.linear.z = float(w)
        # msg.angular.z = 1.0
        

        self.twist_pub.publish(msg)
    
    def get_param(self, param: str):
        request = ParamGet.Request()
        request.param_id = param
        future = self.mavros_get_param_cli.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def set_param(self, param: str, value_integer: int, value_real: float):
        request = ParamSet.Request()
        request.param_id = param
        request.value.integer = value_integer
        request.value.real = value_real
        future = self.mavros_set_param_cli.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def state_callback(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    
    def odom_callback(self, msg: Odometry):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def follow_wp(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            self.disarm()
            self.get_logger().info('Robot Disarmed')
            return
        else: 
            self.get_logger().info('Starting follow waypoints')
            if not self.connected:
                self.get_logger().info('Robot not connected')
                return
            if self.mode != "GUIDED":
                self.change_mode("GUIDED")
                return
            if not self.armed:
                self.arm(True)
                return
        
        self.get_logger().info(f'Moving to waypoint {self.current_wp}: {self.waypoints[self.current_wp]}')
        x, y, z = self.waypoints[self.current_wp]
        vx, vy, vz = self.compute_velocity(x, y, z)

        self.move(x, y, z, vx, vy, vz)

        if self.current_pose is not None:
            distance = ((self.current_pose[0] - x) ** 2 + (self.current_pose[1] - y) ** 2 + (self.current_pose[2] - z) ** 2) ** 0.5
            if distance < 0.5:  # Threshold to consider the waypoint reached
                self.get_logger().info(f'Waypoint {self.current_wp} reached.')
                self.current_wp += 1

    def compute_velocity(self, x, y, z):
        k = 0.15  # Proportional gain

        dx = x - self.current_pose[0]
        dy = y - self.current_pose[1]
        dz = z - self.current_pose[2]

        vx = k * dx
        vy = k * dy
        vz = k * dz

        return vx, vy, vz
    
    def convert_quaternion_to_euler(self):
        qx, qy, qz, qw = self.orientation
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        return roll
    
    def compute_yaw(self, x, y):
        dx = x - self.current_pose[0]
        dy = y - self.current_pose[1]

        target_yaw = math.atan2(dy, dx)
        current_yaw = self.convert_quaternion_to_euler()

        yaw_error = target_yaw - current_yaw

        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        k_yaw = 10.0
        wz = k_yaw * yaw_error

        wz = max(min(wz, 5.0), -5.0)

        return wz, yaw_error
    
    def disarm(self):
        if not self.connected:
            return
        if self.armed:
            self.arm(False)
            self.get_logger().info("Robot disarmed")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.armed:
            node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()