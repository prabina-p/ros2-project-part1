import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from rclpy.qos import ReliabilityPolicy, QoSProfile


class Exercise1_1(Node):

    def __init__(self):
        super().__init__('exercise1_1')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber
        self.timer_period = 0.5
        self.laser_right = 0
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self,msg):
        # Save the laser scan info at 90°
        self.laser_right = msg.ranges[90] 
        # print the data
        

    def motion(self):
        self.get_logger().info('I receive: "%s"' % str(self.laser_right))
        
        # Need to get to the wall; approach slowly
        if self.laser_right > 0.3:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.0
        # Move forward
        elif self.laser_right <= 0.3 and self.laser_right >= 0.2:
            self.cmd.linear.x = 0.04
            self.cmd.angular.z = -0.1
        # Too close to the wall so move away
        elif self.laser_right < 0.2:
            self.cmd.linear.x = 0.04
            self.cmd.angular.z = 0.1

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
