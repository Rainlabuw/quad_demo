%% Initialize Script
clear; clc; close all;
args.KsOmega = [ 200, 400, 2.5; 200, 400, 2.5; 120, 16.7, 0 ];
args.KsTheta = [ 6, 3, 0; 6, 3, 0; 6, 1, 0.35 ];
args.KsVel = [ 25, 1, 0; 25, 1, 0; 25, 15, 0 ];
args.KsPos = [ 2, 0, 0; 2, 0, 0; 2, 0.5, 0 ];
sim = droneSimulator_kaf( "Output/0_" );
%% 1) Simple Loop
trajFcn =  @( t ) [ sin( 2 * pi * t / 5 ); cos( 2 * pi * t / 5 ); 0.5 ];
sim.runSim( args, 10, zeros( 12, 1 ), trajFcn );
sim.printSim( "Output/1_" );
%% 2) Take off & Land
sim.runSim( args, 5, zeros( 12, 1 ), @simpleFlightDemo, @distFnc );
sim.printSim( "Output/2_", getVerif( 4, "Physical/2_takeoff_land.csv" ) );
%% 3) Two Waypoint Trajectory
sim.runSim( args, 10, zeros( 12, 1 ), @positionTrajectoryDemo1, @distFnc );
sim.printSim( "Output/3_", getVerif( 10, "Physical/3_waypoint.csv", [ -0.05, 0.03, -0.04 ] ) );
%% 4) Four Corners Trajectory
sim.runSim( args, 14, zeros( 12, 1 ), @positionTrajectoryDemo2, @distFnc );
sim.printSim( "Output/4_", getVerif( 14, "Physical/4_corners.csv", [ -0.05, 0.05, -0.04 ] ) );
%% 5) Figure 8
sim.runSim( args, 13, zeros( 12, 1 ), @figure8Demo, @distFnc );
sim.printSim( "Output/5_", getVerif( 11, "Physical/5_figure8.csv", [ 0, 0, -0.04 ] ) );
%% 6) Two Waypoint Trajectory
% sim.runSim( args, 1.3, zeros( 12, 1 ), @positionTrajectoryDemo1, @distFnc );
% sim.printSim( "Output/8_", getVerif( 0.9, "Physical/8_waypoint_uheavy.csv", [ 0, -0.03, -0.04 ] ) );
% sim.runSim( args, 10, zeros( 12, 1 ), @positionTrajectoryDemo1, @distFnc );
% sim.drone.m = sim.drone.m + 0.003158;
% sim.printSim( "Output/7_", getVerif( 10, "Physical/7_waypoint_heavy.csv", [ -0.1, 0.05, -0.04 ] ) );
% sim.runSim( args, 2.5, [0.1;0;0;0;0;0;0;0;0;0;0;0], @positionTrajectoryDemo1, @distFnc );
% sim.printSim( "Output/6_", getVerif( 7.3, "Physical/6_waypoint_flippedX.csv", [ 0.05, -0.08, -0.04 ] ) );
% sim.runSim( args, 8, zeros( 12, 1 ), @( t ) t * [ sin( 2 * pi * t / 5 ); cos( 2 * pi * t / 5 ); 2 ] / 8, @distFnc2 );
% sim.printSim( "Output/99_" );

%% Utility Functions
function verification = getVerif( time, filePath, trim )
    table = readtable( filePath );
    table = table( strcmp( table.entity, "vicon-crazyflie8" ), : );
    table.time = table.time - ( max( table.time ) - time );
    table = table( table.time >= 0, : );
    theta = 2 * acos( table.w );
    unitvec = [ table.p, table.r, table.t ] ./ sin( theta / 2 );
    axial = theta .* unitvec;
    dt = diff( table.time );
    veloc = [ table.x, table.y, table.z ];
    for i = 1:3
        veloc( :, i ) = [ lowpass( diff( veloc( :, i ) ) ./ dt, 1e-3 ); 0 ];
    end
    verification = [ table.time, table.x, table.y, table.z, veloc, axial ];
    if exist( "trim", "var" )
         for i = 1:3
             verification( :, i + 1 ) = verification( :, i + 1 ) + trim( i );
         end
    end
end

function disturbance = distFnc2( t, y )
    if 4.2 <= t && t <= 4.6
        disturbance = [ 0; -4.4; -14.3; 0; -0.11; 0 ];
    else
        disturbance = [ 0; 0; -9.8; 0; 0; 0 ];
    end
end

function disturbance = distFnc( t, y )
    if y( 3 ) < 0
        disturbance = [ 0; 0; 0; 0; 0; 0 ];
    else
        disturbance = [ 0; 0; -9.8 * ( 1 - exp( -8 * y( 3 ) ) ); 0; 0; 0 ];
    end
end

function xyz = simpleFlightDemo( t )
    if 0 <= t && t < 2
        xyz = [ 0; 0; 0.5 * ( t / 2 ) ];
    elseif 2 <= t && t < 4
        xyz = [ 0; 0; 0.5 - 0.5 * ( ( t - 2 ) / 2 ) ];
    else
        xyz = [ 0; 0; 0 ];
    end
end

function xyz = figure8Demo( t )
    trajectory = [ ...
        [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]; ...
        [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
        [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],; ...
    ];
    duration = sum( trajectory( :, 1 ) );
    if 0 <= t && t < 2
        xyz = [ 0; 0; 0.8 * ( t / 2 ) ];
    elseif 2 <= t && t < duration + 2
        x0 = [ 0; 0; 0.8 ];
        ta = 2;
        for i = 1:size( trajectory, 1 )
            ta = ta + trajectory( i, 1 );
            if ta > t
                t = t - ( ta - trajectory( i, 1 ) );
                xyz = [ polyval( flip( trajectory( i, 2:9 ) ), t ); polyval( flip( trajectory( i, 10:17 ) ), t ); ...
                       polyval( flip( trajectory( i, 18:25 ) ), t ) ] * 0.85 + x0;
                return;
            end
        end
        xyz = x0;
    elseif 2 + duration <= t && t < 4 + duration
        xyz = [ 0; 0; 0.8 - 0.8 * ( ( t - 2 - duration ) / 2 ) ];
    else
        xyz = [ 0; 0; 0 ];
    end
end

function xyz = positionTrajectoryDemo1( t )
    xyz = executeWaypoints( t, [ 2, 0, 0, 0; 2, 0, 0, 0.5; 2, 0.5, 0.25, 1; ...
        2, -0.25, -0.5, 0.5; 2, 0, 0, 0.5; 2, 0, 0, 0 ] );
end

function xyz = positionTrajectoryDemo2( t )
    xyz = executeWaypoints( t, [ 2, 0, 0, 0; 2, 0, 0, 0.5; 2, 0.25, 0.25, 0.5; ...
        2, 0.25, -0.25, 0.5; 2, -0.25, -0.25, 0.5; 2, -0.25, 0.25, 0.5; 2, 0, 0, 0.5; 2, 0, 0, 0 ] );
end

function xyz = executeWaypoints( t, waypoints )
    ta = 0;
    for i = 1:( size( waypoints, 1 ) - 1 )
        ta = ta + waypoints( i, 1 );
        if ta > t
            t = t - ( ta - waypoints( i, 1 ) );
            xyz = ( ( waypoints( i + 1, 2:4 ) - waypoints( i, 2:4 ) ) / waypoints( i, 1 ) * t + waypoints( i, 2:4 ) )';
            return;
        end
    end
    xyz = [ 0; 0; 0 ];
end