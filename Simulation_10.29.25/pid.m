%% PID Controller Implementation
% Author: Kent Fukuda
% Last Updated: 10/25/2025

classdef pid < handle
    properties
        Kp; Ki; Kd; Kff; outlimit; intlimit; derlimit; modula;
        integ; deriv; error; desired; prevmeas;
        paramlpB1; paramlpB2; paramlpDeriv; datalp;
    end
    methods
        function output = pidStep( this, desired, measured, dt )
            this.desired = desired;
            this.error = this.desired - measured;
            if this.error > this.modula / 2
                this.error = this.error - this.modula;
            elseif this.error < -this.modula / 2
                this.error = this.error + this.modula;
            end
            this.deriv = ( this.prevmeas - measured ) / dt; % avoid derivative kick
            this.prevmeas = measured;
            if isnan( this.deriv )
                this.deriv = 0;
            else
                datalp0 = this.deriv - this.paramlpB1 * this.datalp( 1 ) - this.paramlpB2 * this.datalp( 2 );
                this.deriv = this.paramlpDeriv * ( datalp0 + 2 * this.datalp( 1 ) + this.datalp( 2 ) );
                this.datalp = [ datalp0, this.datalp( 1 ) ];
            end
            this.integ = this.integ + this.error * dt;
            if this.integ > this.intlimit
                this.integ = this.intlimit;
            elseif this.integ < -this.intlimit
                this.integ = -this.intlimit;
            end
            output = this.Kp * this.error + this.Ki * this.integ + this.Kd * this.deriv + this.Kff * this.desired;
            if output > this.outlimit
                output = this.outlimit;
            elseif output < -this.outlimit
                output = -this.outlimit;
            end
        end
        function this = pid( Kp, Ki, Kd, Kff, outlimit, intlimit, derlimit, modula )
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kff = Kff;
            this.outlimit = outlimit;
            this.intlimit = intlimit;
            this.derlimit = derlimit;
            this.modula = modula;
            this.integ = 0;
            this.deriv = 0; 
            this.error = 0; 
            this.desired = 0; 
            this.prevmeas = nan( 1, 1 );
            r = tan( pi / this.derlimit );
            this.paramlpDeriv = r * r / ( 1 + sqrt( 2 ) * r + r * r );
            this.paramlpB1 = this.paramlpDeriv * ( 2 - 2 / ( r * r ) );
            this.paramlpB2 = this.paramlpDeriv * ( 1 / ( r * r ) - sqrt( 2 ) / r + 1  );
            this.datalp = [ 0, 0 ];
        end
    end
end