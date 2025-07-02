import rclpy
import threading
import time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from example_interfaces.msg import String as RosString




class client_node(Node):
    def __init__(self):
        super().__init__('client')
        all_topics = self.get_topic_names_and_types()
        odom_topics = [name for name, types in all_topics
                       if name.endswith('/odom')
                       and 'nav_msgs/msg/Odometry' in types]
        ## create name list of quads
        self.names = [t.lstrip('/').rsplit('/', 1)[0] for t in odom_topics]


        self.subs = {}
        self.pubs = {}
        for cf in self.names:
            self.subs[cf] = self.create_subscription(Odometry,
                                                     cf+"/odom",
                                                     self.subscriber,10)
            self.pubs[cf] = self.create_publisher(PoseStamped,
                                                  cf+"/cmd_pos",1)
        dt = 0.01
        self.timer = self.create_timer(self.dt, self.publisher)


    def subscriber(self,msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation


    def publisher(self):
        for cf in self.names:
            msg = PoseStamped()
            ## set the time stamp for the frames
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cf
            ## set the position
            msg.pose.position.x = float(0.0)
            msg.pose.position.y = float(0.0)
            msg.pose.position.z = float(0.0)
            ## set the attitude
            msg.pose.orientation.x = float(0.0)
            msg.pose.orientation.y = float(0.0)
            msg.pose.orientation.z = float(0.0)
            msg.pose.orientation.w = float(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = client_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

