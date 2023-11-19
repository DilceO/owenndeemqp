classdef Traj_Planner
    properties
        robot
    end

    methods
        function A = cubic_traj(~, p0_pos, p1_pos, p0_vel, p1_vel, t1)
            
            syms a0 a1 a2 a3 a4 a5 a6 a7 a8;
            A = [a0 a1 a2; a3 a4 a5; a6 a7 a8];

            t0 = 0;                             % starting time
            traj_base = [1 t0 t0^2  t0^3;       % trajectory base equations
                         0 1  2*t0  3*t0^2;
                         1 t1 t1^2  t1^3;
                         0 1  2*t1  3*t1^2];

            for i = 1:3            % Solves for eqch point
                traj_result =[p0_pos(i);
                              p0_vel(i);
                              p1_pos(i);
                              p1_vel(i)];
                A(i,:) = linsolve(traj_base, traj_result);  % Returns array of trajectory variables
            end
            
        end

        function A = quintic_traj(~, p0_pos, p1_pos, p0_vel, p1_vel, p0_accel, p1_accel, t1)
            
            syms a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13 a14 a15 a16 a17 a18 a19 a20 a21 a22 a23;
            A = [a0 a1 a2 a3 a4 a5; a6 a7 a8 a9 a10 a11; a12 a13 a14 a15 a16 a17];

            t0 = 0;                             % starting time
            traj_base = [1 t0 t0^2  t0^3   t0^4    t0^5;       % trajectory base equations
                         0 1  2*t0  3*t0^2 4*t0^3  5*t0^4;
                         0 0  2     6*t0   12*t0^2 20*t0^3;
                         1 t1 t1^2  t1^3   t1^4    t1^5;
                         0 1  2*t1  3*t1^2 4*t1^3  5*t1^4;
                         0 0  2     6*t1   12*t1^2 20*t1^3];

            for i = 1:3            % Solves for eqch point
                traj_result =[p0_pos(i);
                              p0_vel(i);
                              p0_accel(i);
                              p1_pos(i);
                              p1_vel(i);
                              p1_accel(i)];
                A(i,:) = linsolve(traj_base, traj_result);  % Returns array of trajectory variables
            end
            
        end
    end
end