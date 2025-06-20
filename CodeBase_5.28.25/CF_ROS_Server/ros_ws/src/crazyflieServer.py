import time
import numpy
import threading
import cflib.crtp as crtp
import motioncapture
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie

from cflib.crazyflie.syncLogger import SyncLogger

class TrackingObject:
    uri = "UNKNOWN"
    object_name = "UNKNOWN"

    def __init__( self, uri, object_name ):
        self.uri = uri
        self.object_name = object_name

class CrazyflieServer:

    def __init__( self, crazyflies=[], vicon_ip="", kill_sleep=20 ):
        self.lock = threading.Lock()
        self.is_open = True
        self.log_data = "entity,time,x,y,z,p,r,t,w\n"
        self.curr_data = {}
        self.threads = []
        self.crazyflies = crazyflies
        print( "1) Connecting to Crazyflies" )
        crtp.init_drivers()
        log_config = LogConfig( name="tlm", period_in_ms=100 )
        log_config.add_variable( 'kalman.stateX', 'float' )
        log_config.add_variable( 'kalman.stateY', 'float' )
        log_config.add_variable( 'kalman.stateZ', 'float' )
        for cf in self.crazyflies:
            print( f"Connecting crazyflie {cf.object_name} to radio: {cf.uri}" )
            cf.cf = Crazyflie()
            cf.cflog = SyncLogger( cf.cf, log_config )
            cf.cf.open_link( cf.uri )
        time.sleep( 1.0 )
        print( "2) Initialzing to Telemetry System" )
        print( f"Connecting to Vicon with IP: {vicon_ip}" )
        self.vicon_ip = vicon_ip
        self.spawn_thread( self.vicon )
        time.sleep( 1.0 )
        print( "3) Initializing Crazyflies Kalman Estimator" )
        kvar_cfg = LogConfig( name="var", period_in_ms=500 )
        kvar_cfg.add_variable( 'kalman.varPX', 'float' )
        kvar_cfg.add_variable( 'kalman.varPY', 'float' )
        kvar_cfg.add_variable( 'kalman.varPZ', 'float' )
        for cf in self.crazyflies:
            cf.cf.param.set_value( 'stabilizer.estimator', '2' )
            cf.cf.param.set_value( 'locSrv.extQuatStdDev', 8.0e-3 )
            cf.cf.param.set_value( 'kalman.resetEstimation', '1' )
            cf.kvar_log = SyncLogger( cf.cf, kvar_cfg )
            cf.var_history = [ [1000] * 10, [1000] * 10, [1000] * 10 ]
            cf.kvar_log.connect()
        print( "4) Awaiting Kalman Estimator" )
        var_active = True
        iteration = 0
        while var_active:
            var_active = False
            for cf in self.crazyflies:
                for log_entry in cf.kvar_log:
                    data = log_entry[1]
                    cf.var_history[0][iteration] = data[ 'kalman.varPX' ];
                    cf.var_history[1][iteration] = data[ 'kalman.varPY' ];
                    cf.var_history[2][iteration] = data[ 'kalman.varPZ' ];
                    threshold = 0.001
                    print( f"Kalman variance: {numpy.amax( cf.var_history ) - numpy.amin( cf.var_history )}" )
                    if ( numpy.amax( cf.var_history ) - numpy.amin( cf.var_history ) ) > threshold :
                        var_active = True
                    break
            iteration = ( iteration + 1 ) % 10
        time.sleep( 1.0 )
        print( "5) Arming Drones" )
        for cf in self.crazyflies:
            cf.cf.param.set_value( 'kalman.resetEstimation', '0')
            cf.kvar_log.disconnect()
        time.sleep( 1.0 )
        for cf in self.crazyflies:
            cf.cflog.connect()
            cf.cf.platform.send_arming_request( True )
            #cf.com = MotionCommander( cf.cf, 0.5 )
        self.spawn_thread( self.logging )
        time.sleep( 1.0 )
        self.kill_sleep = kill_sleep
        self.spawn_thread( self.killswitch )
        print( "6) All ready!" )

    def spawn_thread( self, func ):
        thread = threading.Thread( target=func )
        thread.start()
        self.threads.append( thread );
    
    def ros_bridge_execute( self, command ):
        print( "CRAZYFLIE SERVER EXECUTING COMMAND: " + command )
        args = command.split( "#" )
        match( args[0] ):
            case "hl_SetPos":
                for cf in self.crazyflies:
                    if cf.object_name == args[1]:
                        ix = float( args[2] )
                        iy = float( args[3] )
                        iz = float( args[4] )
                        iyaw = float( args[5] )
                        duration = float( args[6] )
                        self.record( f"{cf.object_name}-sp_SetPos", ix, iy, iz )
                        cf.cf.high_level_commander.go_to( x=ix, y=iy, z=iz, yaw=iyaw, duration_s=duration, relative=False, linear=False, group_mask=0 )
            case "hl_TakeOffAll":
                for cf in self.crazyflies:
                    cf.cf.high_level_commander.takeoff( 0.5, 2.0 )
                time.sleep( 2.0 )
            case "hl_LandAll":
                for cf in self.crazyflies:
                    cf.cf.high_level_commander.land( 0.0, 2.0 )
                time.sleep( 2.0 )
            case "sp_SetPos":
                for cf in self.crazyflies:
                    if cf.object_name == args[1]:
                        self.record( f"{cf.object_name}-sp_SetPos", float( args[2] ), float( args[3] ), float( args[4] ) )
                        cf.cf.commander.send_position_setpoint( float( args[2] ), float( args[3] ), float( args[4] ), float( args[5] ) )
            case "sp_LandAll":
                pos_data = self.current()
                for cf in self.crazyflies:
                    cf_pos = pos_data[ "vicon-" + cf.object_name ][1:4]
                    cf.cf.commander.send_position_setpoint( cf_pos[0], cf_pos[1], float( args[1] ), 0 )
                time.sleep( 1 )
                for cf in self.crazyflies:
                    cf.cf.commander.send_stop_setpoint()
            case "spKill":
                for cf in self.crazyflies:
                    cf.cf.commander.send_stop_setpoint()

    def current( self ):
        return self.curr_data

    def record( self, name, x, y, z, p=0, r=0, t=0, w=0 ):
        t = time.time()
        line = f"{name},{t},{x},{y},{z},{p},{r},{t},{w}";
        self.lock.acquire()
        self.log_data += ( line + "\n" )
        self.curr_data[name] = [ t, x, y, z, p, r, t, w ]
        self.lock.release()

    def vicon( self ):
        mocap = motioncapture.connect( "vicon", { 'hostname': self.vicon_ip } )
        while self.is_open:
            mocap.waitForNextFrame()
            pos_data = mocap.rigidBodies.items()
            for name, obj in pos_data:
                self.record( "vicon-" + name, obj.position[0], obj.position[1], obj.position[2], obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w )
                for cf in self.crazyflies:
                    if name == cf.object_name:
                        cf.cf.extpos.send_extpose( obj.position[0], obj.position[1], obj.position[2], obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w )
    
    def logging( self ):
        while self.is_open:
            for cf in self.crazyflies:
                for log_entry in cf.cflog:
                    data = log_entry[1]
                    self.record( "kalman-" + cf.object_name, data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'] );
                    break

    def killswitch( self ):
        start_time = time.time()
        while self.is_open:
            time.sleep( 1.0 )
            if time.time() - start_time > self.kill_sleep:
                break
        if self.is_open:
            print( f"Shutting down crazyflies due to kill switch timeout" )
            for cf in self.crazyflies:
                cf.cf.high_level_commander.stop()
    
    def close( self ):
        print( "7) Cleaning Up" )
        self.is_open = False
        some_alive = True
        while some_alive:
            some_alive = False
            for thread in self.threads:
                some_alive = some_alive or thread.is_alive()
        path = f"CF_demo_run_{round( time.time() )}.csv";
        file = open( path, "w" )
        file.write( self.log_data )
        file.close()
        print( f"Saved log data to: {path}" )
        for cf in self.crazyflies:
            print( f"Shutting down crazyflie {cf.object_name}" )
            cf.cf.high_level_commander.stop()
            cf.cf.close_link()