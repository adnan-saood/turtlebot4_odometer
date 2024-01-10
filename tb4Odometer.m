classdef tb4Odometer
    %TB4ODOMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        State_
        P
        Q
        r
        L
        R
        estimated_pose = [0 0 0 0];

        timestamp;

        msgs = struct('timestamp', 0,...
                      'ticks_left', 0,...
                      'ticks_right', 0,...
                      'velocity_left', 0,...
                      'velocity_right', 0,...
                      'ax', 0,...
                      'wz', 0,...
                      'dt', 0);
         
    end

    properties (Access = public)
        prev_ticks_left = 0;
        prev_ticks_right = 0;
        isFirst = true;

        delta_ticks_left_prev = 0;
        delta_ticks_right_prev = 0;

        V = 0;
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

            obj.R = ones(5)*1e-3;
        end

        function obj = setQ(obj, Qin)
            obj.Q = Qin;
        end

        function obj = setR(obj, Rin)
            obj.R = Rin;
        end

        function [obj, dS, newP] = predict_(obj, delta_phi_L, delta_phi_R, dt)
            delta_phi_avg = (delta_phi_R + delta_phi_L) / 2.0;
            V_ = obj.r * delta_phi_avg / dt;

            theta = obj.State_(3);
            omega = obj.State_(5);

            V_dt_cos_theta = V_ * dt * cos(theta);
            V_dt_sin_theta = V_ * dt * sin(theta);

            F = [1, 0, -V_dt_sin_theta, dt * cos(theta), 0;
                 0, 1,  V_dt_cos_theta, dt * sin(theta), 0;
                 0, 0, 1, 0, dt;
                 0, 0, 0, 1, 0;
                 0, 0, 0, 0, 1];

            dS = [V_dt_cos_theta; 
                  V_dt_sin_theta; 
                  omega * dt; 
                  0 ; 
                  0];

            newP = F * obj.P * F' + obj.Q;
        end

        function out = pose(obj)
            out = obj.estimated_pose;
        end

        
        function obj = update(obj, new_msg)
          obj.msgs = new_msg;
          obj.timestamp = obj.msgs.timestamp;
          if(obj.isFirst)
            obj.delta_ticks_left_prev = obj.msgs.ticks_left;
            obj.delta_ticks_right_prev = obj.msgs.ticks_right;
            obj.isFirst = false;
          end

          delta_phi_L = obj.msgs.ticks_left - obj.delta_ticks_left_prev;
          delta_phi_R = obj.msgs.ticks_right - obj.delta_ticks_right_prev;


          delta_phi_L = delta_phi_L / 508.8 * 2.0 * pi;
          delta_phi_R = delta_phi_R / 508.8 * 2.0 * pi;


          obj.delta_ticks_left_prev = obj.msgs.ticks_left;
          obj.delta_ticks_right_prev = obj.msgs.ticks_right;

          T_base_IMU = [0.050613 ; 0.043673 ; 0.0202+0.0642];

          omega = [0 ; 0 ; obj.msgs.wz];
          a_additional = cross(omega, cross(omega, T_base_IMU));

          imu_acc = [obj.msgs.ax ; 0 ; 0] - a_additional;

          a_eff = imu_acc - a_additional;

          a_x = a_eff(1);
          a_y = a_eff(2);
          omega_z = omega(3);

          obj.V = obj.V + a_x * obj.msgs.dt;
          obj.theta_imu = obj.theta_imu + omega_z * obj.msgs.dt;


          V_l = obj.msgs.velocity_left;
            V_r = obj.msgs.velocity_right;

            z = [V_l;
                 V_r;
                 obj.theta_imu;
                 obj.V;
                 omega_z];

            [obj, dS, newP] = predict_(obj,delta_phi_L, delta_phi_R, obj.msgs.dt);
            obj.State_ = obj.State_ + dS;
            obj.P = newP;


            H = [0, 0, 0, 1,  obj.L / 2.0;
                    0, 0, 0, 1, -obj.L / 2.0;
                    0, 0, 1, 0, 0;
                    0, 0, 0, 1, 0;
                    0, 0, 0, 0, 1];

            y = z-H*obj.State_;
            S = H*obj.P*H'+obj.R;
            K = obj.P*H'/S;
            obj.State_ = obj.State_+K*y;
            obj.P = (eye(5)-K*H)*obj.P;

            obj.estimated_pose = [obj.State_(1), obj.State_(2), obj.State_(3), obj.timestamp];
        end
    end
end

