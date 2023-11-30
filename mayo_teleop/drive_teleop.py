import rclpy
import threading
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from rclpy.qos import ReliabilityPolicy, QoSProfile

class DriveTeleop(Node):

    def __init__(self):
        super().__init__('drive_teleop')
        self.joint_publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.joint_publisher_1 = self.create_publisher(JointState, 'joint_command_vel', 10)
        self.subscriber_2 = self.create_subscription(Joy, '/joy1', self.handle_keyboard_input, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #thread
        self.lock = threading.Lock()
        self.input_thread = threading.Thread(target=self.handle_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        

        # cmd_vel_nav command
        self.speed_setting = 2 # default to medium speed 2
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

        #Mecanum Angles Doubles 
        # make an array 
        self.wheel_0_ang=np.pi/4
        self.wheel_1_ang=-np.pi/4
        self.wheel_2_ang=-np.pi/4
        self.wheel_3_ang=np.pi/4

        #Velocities for each Wheel
        #remove twist -> make vector 
        self.speed_wheel0=0.0 #LR Inferior Izquierda
        self.speed_wheel1=0.0 #RR Inferior derecha
        self.speed_wheel2=0.0#RF Delantera derecha
        self.speed_wheel3=0.0#LF Delantera izquierda
        
        #Wheel Orientations
        self.wheelO_0=np.array([1.0,0.0])
        self.wheelO_1=np.array([1.0,0.0])
        self.wheelO_2=np.array([1.0,0.0])
        self.wheelO_3=np.array([1.0,0.0])

        #Wheel Positions
        self.wheelP_0=np.array([0.71721,0.555])
        self.wheelP_1=np.array([0.71721,-0.555])
        self.wheelP_2=np.array([-0.71721,0.555])
        self.wheelP_3=np.array([-0.71721,-0.555])
        self.wheel_l1=0.71721
        self.wheel_l2=0.555

        #Wheel Radious
        #Same size for 4 
        self.wheels_rad=0.2

    def kinematic_vel(self, Vx, Vy, omega):
        self.omega_lf = 1 / self.wheels_rad * (Vx - Vy - (self.wheel_l1 + self.wheel_l2) * omega)
        self.omega_rf = 1 / self.wheels_rad * (Vx + Vy + (self.wheel_l1 + self.wheel_l2) * omega)
        self.omega_lr = 1 / self.wheels_rad * (Vx + Vy - (self.wheel_l1 + self.wheel_l2) * omega)
        self.omega_rr = 1 / self.wheels_rad * (Vx - Vy + (self.wheel_l1 + self.wheel_l2) * omega)
        
        
        return self.omega_lr, self.omega_rr, self.omega_rf, self.omega_lf


    def handle_keyboard_input(self):
        while rclpy.ok():
            with self.lock:
                # Set speed ratio using 1,2,3
                adjust_speed_input = input("op: ")
                if adjust_speed_input == '1':
                    self.speed_setting = 0.5
                    print("Low speed.")
                elif adjust_speed_input == '2':
                    self.speed_setting = 1
                    print("medium speed.")
                elif adjust_speed_input == '3':
                    self.speed_setting = 2
                    print("High speed.")
                    ##If the speed exceeds 3, there may be a phase shift in the velocities.
                                
                #Velocity commands
                # Lifting mechanism control
                if 'w' in adjust_speed_input:
                    for _ in range(20):  
                        if self.lifting_mechanism_position < self.lifting_mechanism_max:
                            self.lifting_mechanism_position += self.lifting_mechanism_speed
                    print("Lifting complete.")
                elif 's' in adjust_speed_input:
                    for _ in range(20):  
                        if self.lifting_mechanism_position > self.lifting_mechanism_min:
                            self.lifting_mechanism_position -= self.lifting_mechanism_speed
                    print("Lowering complete.")
                
                # Left Fork positioner control
                if 'd' in adjust_speed_input:
                    for _ in range(20):  
                        if self.fork_positioner_position < self.fork_positioner_max:
                            self.fork_positioner_position += self.fork_positioner_speed
                    print("Fork positioner moving to the right.")
                elif 'a' in adjust_speed_input:
                    for _ in range(20):  
                        if self.fork_positioner_position > self.lifting_mechanism_min:
                            self.fork_positioner_position -= self.fork_positioner_speed
                    print("Fork positioner moving to the left.")

                #tilt
                if adjust_speed_input == 't':
                    self.tilt_position = self.tilt_min
                    print("Tilting.")
                elif adjust_speed_input == 'r':
                    self.tilt_position = 0.0
                    print("Resetting tilt.")

                #Velocity
                if adjust_speed_input == 'y':
                    #All positive move foward x
                    for _ in range(20):
                        self.linear_x_speed = 0.5 *self.speed_setting
                        self.linear_y_speed = 0*self.speed_setting
                        self.angular_z_speed = 0*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                    print('velocidad rueda')
                    print(self.speed_wheel0)
                    #Calcular velocidad de las ruedas con la funcion
                    
                if adjust_speed_input == 'u':
                    #All negative move backward -x ok
                    for _ in range(20):
                    
                        self.linear_x_speed = -0.5 *self.speed_setting
                        self.linear_y_speed = 0.0*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)

                    
                if adjust_speed_input == 'j':
                    #move left positive 
                    for _ in range(20):
                        self.linear_x_speed = 0.0 *self.speed_setting
                        self.linear_y_speed = 0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                    
                if adjust_speed_input == 'k':
                    #move left negative 
                    for _ in range(20):
                        self.linear_x_speed = 0.0 *self.speed_setting
                        self.linear_y_speed = -0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)

                    
                if adjust_speed_input == 'l':
                    #right diagonal foward
                    for _ in range(20):
                        self.linear_x_speed = 0.5 *self.speed_setting
                        self.linear_y_speed = 0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0,self.speed_wheel1, self.speed_wheel2,self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel1=0.0
                        self.speed_wheel3=0.0
                if adjust_speed_input == 'm':
                    #rigth diogonal backwards
                    for _ in range(20):
                        self.linear_x_speed = -0.5 *self.speed_setting
                        self.linear_y_speed = 0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0,self.speed_wheel1, self.speed_wheel2,self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel2=0.0
                        self.speed_wheel0=0.0
                if adjust_speed_input == 'v':
                    #move right  diagonal foward
                    for _ in range(20):
                        self.linear_x_speed = 0.5 *self.speed_setting
                        self.linear_y_speed = -0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0,self.speed_wheel1, self.speed_wheel2,self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel2=0.0
                        self.speed_wheel0=0.0
                if adjust_speed_input == 'b':
                    #left  diagonal backward
                    for _ in range(20):
                        self.linear_x_speed = -0.5 *self.speed_setting
                        self.linear_y_speed = -0.5*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0,self.speed_wheel1, self.speed_wheel2,self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel1=0.0
                        self.speed_wheel3=0.0

                if adjust_speed_input == 'z':
                    #Stop
                    for _ in range(20):
                        self.linear_x_speed = 0.0 *self.speed_setting
                        self.linear_y_speed = 0.0*self.speed_setting
                        self.angular_z_speed = 0.0*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                if adjust_speed_input == 'i':
                    # Move in his own axis +z
                    for _ in range(20):    
                        self.linear_x_speed = 0.0 *self.speed_setting
                        self.linear_y_speed = 0.0*self.speed_setting
                        self.angular_z_speed= 0.5*self.speed_setting
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel0=0.0 
                        self.speed_wheel1=0.0

                  
                    
                if adjust_speed_input == 'h':
                    #Move in his own axis -z negative
                    for _ in range(20):
                        self.linear_x_speed = 0.0 *self.speed_setting
                        self.linear_y_speed = 0.0*self.speed_setting
                        self.angular_z_speed = -0.5*self.speed_setting
                        
                        self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3 = self.kinematic_vel(self.linear_x_speed, self.linear_y_speed, self.angular_z_speed)
                        self.speed_wheel2=0.0 
                        self.speed_wheel3=0.0
            # joint_state
            joint_state_position = JointState()
            joint_state_position.name = ["SliderLift", "SliderForkR", "SliderForkL", "SliderTiltL"]
            joint_state_position.position = [self.lifting_mechanism_position/100, self.fork_positioner_position/100, self.fork_positioner_position/100, self.tilt_position/100]
            self.joint_publisher_.publish(joint_state_position)
       
            #Wheels joint_state
            joint_state_position_W = JointState()
            joint_state_position_W.name = ["RevoluteWheelLR", "RevoluteWheelRR", "RevoluteWheelRF", "RevoluteWheelLF"]
            joint_state_position_W.velocity = [self.speed_wheel0, self.speed_wheel1, self.speed_wheel2, self.speed_wheel3]
            self.joint_publisher_1.publish(joint_state_position_W)

            self.linear_x_speed=0.0
            self.linear_y_speed=0.0
            self.linear_y_speed=0.0

    
            
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



























