import rclpy
# import the ROS2 python dependencies
from rclpy.node import Node

# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist

# import the LaserScan module from sensor_msgs dependencies
from sensor_msgs.msg import LaserScan

from rclpy.qos import ReliabilityPolicy, QoSProfile


class Exercise1_1(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise1_1')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber

        # define the timer period for 0.5 seconds
        self.timer_period = 0.5

        # define the variables to save the received info
        self.front_laser_left = 0.0
        self.front_laser_right = 0.0
        self.min_right_laser = 0.0
        self.max_right_laser = 0.0

        # create a Twist message
        self.cmd = Twist()
        
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
        # Scans the front-right area for the closest obstacle
        self.front_laser_right = min(msg.ranges[170:180])
        # Scans the front-left area for the closest obstacle
        self.front_laser_left = min(msg.ranges[180:190])
        # Scans the right area for the closest obstacle
        self.min_right_laser = min(msg.ranges[75:105])
        # Scans the right area for the furthest obstacle
        self.max_right_laser = max(msg.ranges[75:105])

    def motion(self):
        # print the data
        self.get_logger().info("Front-Right Laser: {0} \n Front-Left Laser: {1} \n Right Min Laser: {2} \n Right Max Laser: {3} \n Linear.X {4}".format(
            self.front_laser_right, self.front_laser_left, self.min_right_laser, self.max_right_laser, self.cmd.linear.x))

        if self.front_laser_right < 0.5:  # Incoming front-right obstacle, so move left
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = 0.7
        elif self.front_laser_right < 0.2:  # Obstacle in front-right is very close, so move hard left
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 1.3
        elif self.front_laser_left < 0.5:  # Incoming obstacle on front-left, so move right
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = -0.7
        elif self.front_laser_left < 0.2:  # Obstacle in front-left is very close, so move hard right
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -1.3
        elif self.max_right_laser > 0.3:  # Far from the right wall, so move right
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = -0.15
        elif self.min_right_laser < 0.2:  # Too close to the right wall, so move left
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.3
        else:  # Perfect distance from right wall, so keep moving forward
            self.cmd.linear.x = 0.15
            self.cmd.angular.z = 0.0

        # Publishing the cmd_vel values to topic
        self.publisher_.publish(self.cmd)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    exercise1_1 = Exercise1_1()

    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(exercise1_1)

    # Explicity destroy the node
    exercise1_1.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()