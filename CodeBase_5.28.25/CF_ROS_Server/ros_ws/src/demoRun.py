import time
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

class TrajectoryGenerator:
    trajectory_id = 1
    trajectory = []
    init_pos = [ 0, 0, 0 ]
    total_duration = 0

    def __init__( self ):
        pass

    def add_traj_pos( self, t, x, y, z ):
        self.add_traj_vel( self, t, ( x - self.init_pos[0] ) / t, ( y - self.init_pos[1] ) / t, ( z - self.init_pos[2] ) / t )

    def add_traj_vel( self, t, vx, vy, vz ):
        row = [0] * 33
        row[0] = t
        row[1] = self.init_pos[0]
        row[9] = self.init_pos[1]
        row[17] = self.init_pos[2]
        row[2] = vx
        row[10] = vy
        row[18] = vz
        self.init_pos[0] += vx * t
        self.init_pos[1] += vy * t
        self.init_pos[2] += vz * t
        self.trajectory.append( row )

    def upload_traj( self, cfs ):
        for cf in cfs:
            trajectory_mem = cf.cf.mem.get_mems( MemoryElement.TYPE_TRAJ )[0]
            trajectory_mem.trajectory = []
            self.total_duration = 0
            for row in self.trajectory:
                duration = row[0]
                x = Poly4D.Poly( row[1:9] )
                y = Poly4D.Poly( row[9:17] )
                z = Poly4D.Poly( row[17:25] )
                yaw = Poly4D.Poly( row[25:33] )
                trajectory_mem.trajectory.append( Poly4D( duration, x, y, z, yaw ) )
                self.total_duration += duration
            trajectory_mem.write_data_sync()
        for cf in cfs:
            cf.cf.high_level_commander.define_trajectory( self.trajectory_id, 0, len( trajectory_mem.trajectory ) )

    def run_traj( self, isRelative, cfs ):
        for cf in cfs:
            cf.cf.high_level_commander.start_trajectory( self.trajectory_id, 1.0, isRelative )
        time.sleep( self.total_duration )

def do_nothing_demo( molog ):
    time.sleep( 10.0 )

def simple_flight_demo( molog ):
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.takeoff( 0.5, 2.0 )
    time.sleep( 2.0 )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.land( 0.0, 2.0 )
    time.sleep( 2.0 )

def figure8_flight_demo( molog ):
    traj = TrajectoryGenerator()
    traj.trajectory = [
        [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
        [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    ]
    traj.upload_traj( molog.crazyflies )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.takeoff( 0.5, 2.0 )
    time.sleep( 2 )
    traj.run_traj( True, molog.crazyflies )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.land( 0.0, 2.0 )
    time.sleep( 2 )

def position_trajectory_demo1( molog ):
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.takeoff( 0.5, 2.0 )
    time.sleep( 2.0 )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.go_to( x=0.5, y=0.25, z=1, yaw=0, duration_s=2, relative=False, linear=False, group_mask=0 )
    time.sleep( 2.0 )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.go_to( x=-0.5, y=-0.5, z=0.5, yaw=0, duration_s=2, relative=False, linear=False, group_mask=0 )
    time.sleep( 2.0 )
    for cf in molog.crazyflies:
        cf.cf.high_level_commander.land( 0.0, 2.0 )
    time.sleep( 2.0 )

def setpoint_demo( molog ):
    for cf in molog.crazyflies:
        cf.cf.commander.send_position_setpoint( 0, 0, 0.5, 0 )
    time.sleep( 1.0 )
    for cf in molog.crazyflies:
        cf.cf.commander.send_position_setpoint( 0, 0, 0, 0 )
    time.sleep( 1.0 )
    for cf in molog.crazyflies:
        cf.cf.commander.send_stop_setpoint()

def crazyflie7Code( molog, cf, pos ):
    print( "A" )
    for i in range(50):
        cf.commander.send_position_setpoint( 0, 0, 0.5, 0 )
        time.sleep( 0.1 )
    for i in range(50):
        cf.commander.send_position_setpoint( 0.5, 0, 0.5, 0 )
        time.sleep( 0.1 )
    for i in range(50):
        cf.commander.send_position_setpoint( 0, -0.5, 0.5, 0 )
        time.sleep( 0.1 )
    for i in range(50):
        cf.commander.send_position_setpoint( 0, 0, 0.5, 0 )
        time.sleep( 0.1 )   
    for i in range(50):
        cf.commander.send_position_setpoint( 0, 0, 0, 0 )
        time.sleep( 1.0 )
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()
    print( "B" )

    """for index in range( 0, 300 ):
        phi = index * 0.01 * ( 2 * numpy.pi )
        if index == 0:
            r = 0
        else:
            r = 0.3
        x = r * numpy.cos( phi )
        y = r * numpy.sin( phi )
        cf.commander.send_position_setpoint( x, y, 0.5, 0 )
        curr_pos = pos()
        print( f"{curr_pos}\t{[ x, y, 0.5 ]}\t{math.dist( curr_pos, [ x, y, 0.5 ] )}" )
        time.sleep( 0.25 )

        while True:
            cf.commander.send_position_setpoint( x, y, 0.5, 0 )
            curr_pos = pos()
            print( f"{curr_pos}\t{[ x, y, 0.5 ]}\t{math.dist( curr_pos, [ x, y, 0.5 ] )}" )
            if( math.dist( curr_pos, [ x, y, 0.5 ] ) < 0.1 ):
                print( "REACHED" )
                break
            time.sleep( 0.25 )"""
def crazyflie8Code( molog, cf, pos ):
    while molog.is_open:
        pass
    
"""
def leader_follower_demo( molog ):  phi = index * 0.01 * ( 2 * numpy.pi )
    if index == 0:
        r = 0
    else:
        r = 0.3
    x = r * numpy.cos( phi )
    y = r * numpy.sin( phi )
    cf.commander.send_position_setpoint( x, y, 0.5, 0 )
    curr_pos = pos()
    print( f"{curr_pos}\t{[ x, y, 0.5 ]}\t{math.dist( curr_pos, [ x, y, 0.5 ] )}" )
    time.sleep( 0.25 )

    while True:
        cf.commander.send_position_setpoint( x, y, 0.5, 0 )
        curr_pos = pos()
        print( f"{curr_pos}\t{[ x, y, 0.5 ]}\t{math.dist( curr_pos, [ x, y, 0.5 ] )}" )
        if( math.dist( curr_pos, [ x, y, 0.5 ] ) < 0.1 ):
            print( "REACHED" )
            break
        time.sleep( 0.25 )

def crazyflie7Code( molog ):
    cf7 = None
    cf8 = None
    for cf in molog.crazyflies:
        if cf.object_name == "crazyflie7":
            cf7 = cf
        if cf.object_name == "crazyflie8":
            cf8 = cf
    newTime = 0
    while cf8.isActive:
        current = molog.current()
        p7 = current[ "vicon-crazyflie7" ][1:4]
        p8 = current[ "vicon-crazyflie8" ][1:4]
        dp = [ p8[0], p8[1], p8[2] - 1 ]
        delta = [ dp[0] - p7[0], dp[1] - p7[1], dp[2] - p7[2] ]
        deltaAmp = float( math.dist( delta, [ 0, 0, 0 ] ) )
        if deltaAmp > 1:
            delta = [ delta[0] / deltaAmp, delta[1] / deltaAmp, delta[2] / deltaAmp ]
        if time.time() > newTime:
            newTime = time.time() + 0.25
            print( f"E7: {p7}\tE8: {p8}\tDest: {dp}\tDiff{delta}" )
        cf8.com.stop()
        cf8.com.start_linear_motion( delta[0], delta[1], delta[2] )
        time.sleep( 0.1 )

def crazyflie8Code( molog ):
    cf8 = None
    for cf in molog.crazyflies:
        if cf.object_name == "crazyflie8":
            cf8 = cf
    cf8.isActive = True
    cf8.cf.high_level_commander.go_to( x=0, y=0.5, z=0.5, yaw=0, duration_s=2, relative=False, linear=False, group_mask=0 )
    time.sleep( 2.0 )
    cf8.cf.high_level_commander.go_to( x=0, y=-0.5, z=0.5, yaw=0, duration_s=2, relative=False, linear=False, group_mask=0 )
    time.sleep( 2.0 )
    cf8.cf.high_level_commander.go_to( x=0, y=0, z=0.5, yaw=0, duration_s=2, relative=False, linear=False, group_mask=0 )
    time.sleep( 2.0 )
    cf8.isActive = False
        
def temp_demo1( molog ):
    for cf in molog.crazyflies:
        cf.com.take_off( 0.5, 0.2 )
    molog.spawn_thread( lambda: crazyflie7Code( molog ) )
    crazyflie8Code( molog )
    for cf in molog.crazyflies:
        cf.com.land( 0.2 ) """

def run_demo( molog ):
    simple_flight_demo( molog )