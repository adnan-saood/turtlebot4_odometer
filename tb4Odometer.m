classdef tb4Odometer
    %TB4ODOMETER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        State_
        P % NxN
        Q % NxN
        r % 1
        L %
        R
        estimated_pose = [0 0 0 0];

        timestamp;

        msgs = struct('timestamp', 0,...
            'ticks_left', 0,...
            'ticks_right', 0,...
            'velocity_left', 0,...
            'velocity_right', 0,...
            'laserX', 0, ...
            'laserY', 0, ...
            'laserTheta', 0, ...
            'ax', 0,...
            'wz', 0,...
            'dt', 0);

    end

    properties (Access = public)
        prev_ticks_left = 0;
        prev_ticks_right = 0;
        isFirst = true;

        ticks_left_prev = 0;
        ticks_right_prev = 0;

        V_imu = 0;
        theta_imu = 0;
    end

    methods
        function obj = tb4Odometer(obj)
            obj.State_ = zeros(5, 1);
            obj.P = eye(5) * 1.0e-9;
            obj.Q = eye(5) * 0.0001;
            obj.r = 0.03575;
            obj.L = 0.235;
            obj.timestamp = 0.0;

            obj.R = ones(8)*1e-3;
        end

        function obj = setQ(obj, Qin)
            obj.Q = Qin;
        end

        function obj = setR(obj, Rin)
            obj.R = Rin;
        end

        function [obj, newP] = predict_(obj, delta_phi_R, delta_phi_L, dt)
            delta_phi_avg = (delta_phi_R + delta_phi_L) / 2.0;
            V_ = obj.r * (delta_phi_R + delta_phi_L) / (2.0*dt);
            delta_S = delta_phi_avg * obj.r;
            delta_Theta = obj.r * (delta_phi_R - delta_phi_L) / obj.L;

            theta = obj.State_(3);
            omega = delta_Theta / (dt);

            F = [1, 0, -delta_S * sin(theta + delta_Theta/2), 0, 0;
                0, 1,  delta_S * cos(theta + delta_Theta/2), 0, 0;
                0, 0, 1, 0, 0;
                0, 0, 0, 1, 0;
                0, 0, 0, 0, 1];

            dV = V_ - obj.State_(4);
            domega = omega - obj.State_(5);
 

            obj.State_ = obj.State_ + ...
                [delta_S * cos(theta + delta_Theta/2);
                 delta_S * sin(theta + delta_Theta/2);
                  delta_Theta;
                  dV;
                  domega];

            if(obj.State_(3) > pi)
                obj.State_(3) = obj.State_(3)-2*pi;
            end
            newP = F * obj.P * F' + obj.Q;
        end

        function out = pose(obj)
            out = obj.estimated_pose;
        end

        function obj = update(obj, new_msg)
            obj.msgs = new_msg;
            obj.timestamp = obj.msgs.timestamp;
            if(obj.isFirst)
                obj.ticks_right_prev = obj.msgs.ticks_right;
                obj.ticks_left_prev = obj.msgs.ticks_left;
                obj.isFirst = false;
            end

            delta_ticks_R = obj.msgs.ticks_right - obj.ticks_right_prev;
            delta_ticks_L = obj.msgs.ticks_left - obj.ticks_left_prev;

            delta_phi_R = delta_ticks_R / 508.8 * 2.0 * pi;
            delta_phi_L = delta_ticks_L / 508.8 * 2.0 * pi;

            obj.ticks_right_prev = obj.msgs.ticks_right;
            obj.ticks_left_prev = obj.msgs.ticks_left;

            T_base_IMU = [0.050613 ; 0.043673 ; 0.0202+0.0642];

            omega = [0 ; 0 ; obj.msgs.wz];
            a_additional = cross(omega, cross(omega, T_base_IMU));

            imu_acc = [obj.msgs.ax ; 0 ; 0] - a_additional;

            a_eff = imu_acc - a_additional;

            a_x = a_eff(1);
            a_y = a_eff(2);
            
            omega_z = omega(3);

            obj.V_imu = obj.V_imu + a_x * obj.msgs.dt;
            obj.theta_imu = obj.theta_imu + omega_z * obj.msgs.dt;
            if(obj.theta_imu > pi)
                obj.theta_imu = obj.theta_imu - 2*pi;
            end

            PhiDot_r = obj.msgs.velocity_right;
            PhiDot_l = obj.msgs.velocity_left;

            laserX = obj.msgs.laserX;
            laserY = obj.msgs.laserY;
            laserTheta = obj.msgs.laserTheta;

            z = [PhiDot_r;
                PhiDot_l;
                obj.V_imu;
                omega_z;
                laserX;
                laserY;
                laserTheta];

            [obj, newP] = predict_(obj, delta_phi_R, delta_phi_L, obj.msgs.dt);
            obj.P = newP;

            H = [0, 0, 0, 1,  obj.L / 2.0;
                 0, 0, 0, 1, -obj.L / 2.0;
                 0, 0, 0, 1, 0;
                 0, 0, 0, 0, 1;
                 1, 0, 0, 0, 0;
                 0, 1, 0, 0, 0;
                 0, 0, 1, 0, 0];

            y = z - H * obj.State_;
            S = H * obj.P *H.' + obj.R;
            K = obj.P * H.' / S;
            obj.State_ = obj.State_ + K * y;
            obj.P = (eye(5) - K * H) * obj.P;

            obj.estimated_pose = [obj.State_(1), obj.State_(2), obj.State_(3), obj.timestamp];
        end
    end
end