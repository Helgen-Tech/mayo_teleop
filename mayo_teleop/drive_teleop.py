import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from rclpy.qos import ReliabilityPolicy, QoSProfile


class DriveTeleop(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('drive_teleop')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.joint_publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # cmd_vel_nav command
        self.speed_setting = 2 # default to medium speed
        self.linear_x_speed = 0
        self.linear_y_speed = 0
        self.angular_z_speed = 0

        # Fork positioner prism
        self.lifting_mechanism_speed = 1
        self.lifting_mechanism_min = -10.0
        self.lifting_mechanism_max = 40.0
        self.lifting_mechanism_position = -10.0

        # Right fork prism & left fork prism
        self.fork_positioner_speed = 1
        self.fork_positioner_min = -20
        self.fork_positioner_max = 5
        self.fork_positioner_position = 0.0

        # Tilting prism
        self.tilt_speed = 1
        self.tilt_min = -5.0
        self.tilt_max = 5.0
        self.tilt_position = 0.0

    def joy_callback(self,msg):
        # Set speed ratio using d-pad
        if msg.axes[3] == -1: # full speed (d-pad up)
            self.speed_setting = 1
        if msg.buttons[11]: # medium speed (d-pad left or right)
            self.speed_setting = 2
        if msg.axes[3] == 1: # low speed (d-pad down)
            self.speed_setting = 3


        # Lifting mechanism control 
        if msg.buttons[0]: 
            if self.lifting_mechanism_position < self.lifting_mechanism_max:
                self.lifting_mechanism_position += self.lifting_mechanism_speed
        if msg.buttons[2]: 
            if self.lifting_mechanism_position > self.lifting_mechanism_min:
                self.lifting_mechanism_position -= self.lifting_mechanism_speed

        # Left Fork positioner control
        if msg.buttons[3]: 
            if self.fork_positioner_position < self.fork_positioner_max:
                self.fork_positioner_position += self.fork_positioner_speed
        if msg.buttons[1]:
            if self.fork_positioner_position > self.lifting_mechanism_min:
                self.fork_positioner_position -= self.fork_positioner_speed

        # Tilt control (direct position command)
        if msg.buttons[5]: 
                self.tilt_position = self.tilt_min
        if msg.buttons[7]:
                self.tilt_position = 0.0

        # Drive sticks
        self.linear_x_speed = msg.axes[1] / self.speed_setting # left stick up/down
        self.linear_y_speed = msg.axes[0] / self.speed_setting # left stick left/right
        self.angular_z_speed = -msg.axes[2] / (3.0 *self.speed_setting) # right stick left/right

        # joint_state
        joint_state_position = JointState()
        joint_state_position.name = ["SliderLift", "SliderForkR", "SliderForkL", "SliderTiltL"]
        joint_state_position.position = [self.lifting_mechanism_position/100, self.fork_positioner_position/100, self.fork_positioner_position/100, self.tilt_position/100]
        self.joint_publisher_.publish(joint_state_position)

        # cmd_vel_nav
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x_speed
        twist_msg.linear.y = self.linear_y_speed
        twist_msg.angular.z = self.angular_z_speed
        self.cmd_publisher_.publish(twist_msg)
        self.get_logger().info('Publishing: "%s"' % twist_msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    drive_teleop = DriveTeleop()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(drive_teleop)
    # Explicity destroys the node
    drive_teleop.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()