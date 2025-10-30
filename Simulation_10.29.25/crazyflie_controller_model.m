%% Crazyflie Cascaded PID Controller Model in MATLAB
% Author: Kent Fukuda
% Last Updated: 10/25/2025

classdef crazyflie_controller_model
    methods( Static )
        function init( args, sim, id )
            KsOmega = args.KsOmega;
            KsTheta = args.KsTheta;
            KsVel = args.KsVel;
            KsPos = args.KsPos;
            tlim = 65535 / 2 / 1000;
            sim.drone.record.angVelPids = ...
                              [ pid( KsOmega( 1, 1 ), KsOmega( 1, 2 ), KsOmega( 1, 3 ), 0, 1e6,  33.3, 500 / 30, 1e6 );%roll
                                pid( KsOmega( 2, 1 ), KsOmega( 2, 2 ), KsOmega( 2, 3 ), 0, 1e6,  33.3, 500 / 30, 1e6 );%pitch
                                pid( KsOmega( 3, 1 ), KsOmega( 3, 2 ), KsOmega( 3, 3 ), 0, 1e6, 166.7, 500 / 30, 1e6 ) ];%yaw
            sim.drone.record.angularPids = ...
                              [ pid( KsTheta( 1, 1 ), KsTheta( 1, 2 ), KsTheta( 1, 3 ), 0, 1e6,  20,   500 / 15, 180 );%roll
                                pid( KsTheta( 2, 1 ), KsTheta( 2, 2 ), KsTheta( 2, 3 ), 0, 1e6,  20,   500 / 15, 180 );%pitch
                                pid( KsTheta( 3, 1 ), KsTheta( 3, 2 ), KsTheta( 3, 3 ), 0, 1e6, 360,   500 / 15, 180 ); ];%yaw
            sim.drone.record.velocPids = ...
                              [ pid( KsVel( 1, 1 ), KsVel( 1, 2 ), KsVel( 1, 3 ), 0, 20 * 1.1, 1e6, 100 / 20, 1e6 );%x
                                pid( KsVel( 2, 1 ), KsVel( 2, 2 ), KsVel( 2, 3 ), 0, 20 * 1.1, 1e6, 100 / 20, 1e6 );%y
                                pid( KsVel( 3, 1 ), KsVel( 3, 2 ), KsVel( 3, 3 ), 0, tlim,     1e6, 100 / 20, 1e6 ) ];%z
            sim.drone.record.posPids = ...
                              [ pid( KsPos( 1, 1 ), KsPos( 1, 2 ), KsPos( 1, 3 ), 0, 1  * 1.1, 1e6, 100 / 20, 1e6 );%x
                                pid( KsPos( 2, 1 ), KsPos( 2, 2 ), KsPos( 2, 3 ), 0, 1  * 1.1, 1e6, 100 / 20, 1e6 );%y
                                pid( KsPos( 3, 1 ), KsPos( 3, 2 ), KsPos( 3, 3 ), 0, 1  * 1.1, 1e6, 100 / 20, 1e6 ) ];%z
            sim.drone.record.attitudeSetpoint = [ 0; 0; 0 ];
            sim.drone.record.thrustSetpoint = 0;
            sim.drone.record.prevAngleEuler = [ 0; 0; 0 ];
        end
        function loopPosition( state, sim, id )
            % setup
            timeStep = sim.drone.s( id ).dt;
            sim.ctrl.desired( 1:3 ) = sim.ctrl.input;
            desiredAttEuler = zeros( 3, 1 );
            desiredVel = zeros( 3, 1 );
            % body coordinate conversion
            attitudeEuler = sim.axial2euler( state( 7:9 ) );
            cosYaw = cos( attitudeEuler( 3 ) * pi / 180 );
            sinYaw = sin( attitudeEuler( 3 ) * pi / 180 );
            currentPos = [ state( 1 ) * cosYaw + state( 2 ) * sinYaw; ...
                           -state( 1 ) * sinYaw + state( 2 ) * cosYaw; state( 3 ) ];
            currentVel = [ state( 4 ) * cosYaw + state( 5 ) * sinYaw; ...
                           -state( 4 ) * sinYaw + state( 5 ) * cosYaw; state( 6 ) ];
            desiredPos = [ sim.ctrl.input( 1 ) * cosYaw + sim.ctrl.input( 2 ) * sinYaw; ...
                           -sim.ctrl.input( 1 ) * sinYaw + sim.ctrl.input( 2 ) * cosYaw; sim.ctrl.input( 3 ) ];
            % pid
            for i = 1:3
                desiredVel( i ) = sim.drone.record.posPids( i ).pidStep( ...
                    desiredPos( i ), currentPos( i ), timeStep );
                desiredAttEuler( i ) = sim.drone.record.velocPids( i ).pidStep( ...
                    desiredVel( i ), currentVel( i ), timeStep );
            end
            sim.ctrl.desired( 4:6 ) = [ desiredVel( 1 ) * cosYaw - desiredVel( 2 ) * sinYaw; ...
                           desiredVel( 1 ) * sinYaw + desiredVel( 2 ) * cosYaw; desiredVel( 3 ) ];
            % attitude setpoint
            sim.drone.record.attitudeSetpoint( 1 ) = -desiredAttEuler( 2 );
            sim.drone.record.attitudeSetpoint( 2 ) = desiredAttEuler( 1 );
            sim.drone.record.attitudeSetpoint( 3 ) = 0;
            sim.drone.record.thrustSetpoint        = max( 1000 * desiredAttEuler( 3 ) + 30000, 20000 );
        end
        function loopAttitude( state, sim, id )
            % setup
            timeStep = sim.drone.s( id ).dt;
            desiredInput = [ 0; 0; 0 ];
            currentAngleEuler = sim.axial2euler( state( 7:9 ) );
            desiredAngleEuler = sim.drone.record.attitudeSetpoint;
            sim.ctrl.desired( 7:9 ) = sim.euler2axial( desiredAngleEuler );
            % angular velocity calculation
            currentAngVel = currentAngleEuler - sim.drone.record.prevAngleEuler;
            currentAngVel( abs( currentAngVel ) > 175 ) = 0;
            currentAngVel = currentAngVel * 500;
            sim.drone.record.prevAngleEuler = currentAngleEuler;
            % pid
            desiredAngVel = zeros( 3, 1 );
            for i = 1:3
                desiredAngVel( i ) = sim.drone.record.angularPids( i ).pidStep( ...
                    desiredAngleEuler( i ), currentAngleEuler( i ), timeStep );
                desiredInput( i ) = sim.drone.record.angVelPids( i ).pidStep( ...
                    desiredAngVel( i ), currentAngVel( i ), timeStep );
            end
            sim.ctrl.desired( 10:12 ) = sim.euler2axial( desiredAngVel );
            % output
            r = desiredInput( 1 ) / 2;
            p = desiredInput( 2 ) / 2;
            y = desiredInput( 3 );
            t = sim.drone.record.thrustSetpoint;
            motorOutput = max( min( [ t - r + p + y; t - r - p - y; t + r - p + y; t + r + p - y ] / 65535, 1 ), 0 );
            sim.ctrl.output = motorOutput( [ 3, 2, 1, 4 ] );
        end
    end
end