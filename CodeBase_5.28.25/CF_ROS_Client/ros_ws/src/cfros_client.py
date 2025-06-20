import rclpy
import threading
import time
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from example_interfaces.msg import String as RosString

client_node = None
is_active = True
cf8_pos = None
cf8_active = True

def manual():
    global is_active
    global client_node
    while is_active:
        print( "Enter command:" )
        str = input()
        if str == "EXIT":
            is_active = False
            client_node.destroy_node()
            rclpy.shutdown()
            break;
        else:
            string_msg = RosString()
            string_msg.data = str
            sendcommand( str ) ##use to be sendcommand( string_msg )

def sendcommand( arg ):
    global client_node
    string_msg = RosString()
    string_msg.data = arg
    client_node.pub.publish( string_msg )
    time.sleep( 2 )

def trackCF8( odom_msg ):
    global cf8_pos
    if( odom_msg.header.frame_id == "crazyflie8" ):
        cf8_pos = odom_msg.pose.pose.position;

def algoCF7():
    time.sleep( 3 )
    while cf8_active:
        sendcommand( f"hl_SetPos#crazyflie7#{cf8_pos.x+0.4}#{cf8_pos.y}#{cf8_pos.z+0.3}#0#2" )

def algoCF8():
    global cf8_active
    cf8_active = True
    sendcommand( "hl_TakeOffAll" )
    sendcommand( "hl_SetPos#crazyflie8#0.3#0.5#0.5#0#2" )
    sendcommand( "hl_SetPos#crazyflie8#-0.7#0.5#0.5#0#2" )
    sendcommand( "hl_SetPos#crazyflie8#-0.7#-0.5#0.5#0#2" )
    sendcommand( "hl_SetPos#crazyflie8#0.3#-0.5#0.5#0#2" )
    sendcommand( "hl_SetPos#crazyflie8#0#0#0.3#0#2" )
    cf8_active = False
    sendcommand( "hl_LandAll" )
    sendcommand( "shutdown" )

def main(args=None):
    global is_active
    global client_node
    print( "CF ROS Initializing!" )
    is_active = True
    rclpy.init( args=args )
    client_node = Node( 'demo_client' )
    client_node.pub = client_node.create_publisher( RosString, '/cf_commands', 10 )
    client_node.create_subscription( Odometry, '/cf_odometry', trackCF8, 1 )
    print( "CF ROS Started!" )
    threading.Thread( target=manual ).start()
    threading.Thread( target=algoCF7 ).start()
    threading.Thread( target=algoCF8 ).start()
    while is_active: rclpy.spin( client_node )

if __name__ == '__main__':
    main()