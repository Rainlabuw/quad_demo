import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from example_interfaces.msg import String as RosString
from .crazyflieServer import CrazyflieServer
from .crazyflieServer import TrackingObject


class crazyfliebridge(Node):
    def __init__(self):
        super().__init__('crazyflie_bridge')
        # Initialize Crazyflie server FIRST
        crazyflies = [
            TrackingObject( "radio://0/80/2M/E7E7E7E7E7", "crazyflie7" ),
            TrackingObject( "radio://0/80/2M/E7E7E7E7E8", "crazyflie8" ),
            #TrackingObject( "radio://0/80/2M/E7E7E7E7E9", "crazyflie9" ),
        ]
        vicon_ip = "192.168.0.39"
        self.CrazyflieServer = CrazyflieServer( crazyflies, vicon_ip, 300 )
        self.is_active = True
        self.pub = self.create_publisher( Odometry, '/cf_odometry', 10 )
        self.create_subscription( TwistStamped, '/cf_mocom', self.mocom, 1 )
        self.create_subscription( RosString, '/cf_commands', self.cmd, 1 )
        self.timer = self.create_timer( 0.02, self.pos )

    def pos( self ):
        current_data = self.CrazyflieServer.current().copy()
        for cf in self.CrazyflieServer.crazyflies:
            vicon_key = "vicon-" + cf.object_name
            if vicon_key in current_data:
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = cf.object_name
                pos_data = current_data[vicon_key]
                odom_msg.pose.pose.position.x = float( pos_data[1] )
                odom_msg.pose.pose.position.y = float( pos_data[2] )
                odom_msg.pose.pose.position.z = float( pos_data[3] )
                odom_msg.pose.pose.orientation.x = float( pos_data[4] )
                odom_msg.pose.pose.orientation.y = float( pos_data[5] )
                odom_msg.pose.pose.orientation.z = float( pos_data[6] )
                odom_msg.pose.pose.orientation.w = float( pos_data[7] )
                self.pub.publish( odom_msg )

    def mocom( self, msg ):
        try:
            command = f"hl_SetPos#{msg.header.frame_id}#{msg.twist.linear.x}#{msg.twist.linear.y}#{msg.twist.linear.z}#{msg.twist.angular.z}"
            self.CrazyflieServer.ros_bridge_execute( command )
        except Exception as e:
            self.get_logger().error(f"Command failed: {str(e)}")
    
    
    def cmd( self, msg ):
        self.get_logger().info( f"Received command {msg.data}" )
        if msg.data == "shutdown":
            self.is_active = False
            self.CrazyflieServer.close()
            self.destroy_node()
            threading.Thread( target=( lambda: rclpy.shutdown() ) ).start()
        else:
            self.CrazyflieServer.ros_bridge_execute( msg.data )


def main(args=None):
    rclpy.init(args=args)
    bridge = crazyfliebridge()
    while bridge.is_active: rclpy.spin( bridge )

if __name__ == '__main__':
    main()