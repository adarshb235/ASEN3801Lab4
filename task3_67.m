clear
clc
close all

%% 3.7
m = 0.068; 
d = 0.060; 
km = 0.0024; 
I = diag([5.8e-5, 7.2e-5, 1.0e-4]);
nu = 1e-3; 
mu = 2e-6;
g = 9.81;

var0 = zeros(12, 1);
tspan = [0 5];

[t_sim, var_sim] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Outer_Nonlinear(t, var, g, m, I, d, km, nu, mu), tspan, var0);

% longitudinal
figure();
subplot(2,1,1)
plot(t_sim, var_sim(:,1))
hold on
grid on
title('X Displacement vs Time')
xlabel('Time (s)')
ylabel('X Position (m)')

subplot(2,1,2);
plot(t_sim, var_sim(:,7))
hold on
grid on
title('X Velocity vs Time')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

% lateral
figure()
subplot(2,1,1)
plot(t_sim, var_sim(:,2))
hold on
grid on
title('Y Displacement vs Time');
xlabel('Time (s)')
ylabel('Y Position (m)')

subplot(2,1,2);
plot(t_sim, var_sim(:,8))
hold on
grid on
title('Y Velocity vs Time')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

%{
The differences occur because a physical quadrotor cannot change speed 
instantly. It first has to pitch or roll to accelerate, causing a natural 
lag. Furthermore, the controller never reaches the full 0.5 m/s target 
because a proportional-only system requires a constant velocity error to 
hold the pitch angle needed to fight aerodynamic drag. Since 
the actual speed ramps up slowly and stays lower than the target, the 
total distance traveled falls short of the expected 1.0 meter. 
Additionally, the forward speed is slightly lower because the drone 
is physically harder to tilt forward than sideways, so the controller 
restricts its forward tilt more.
%}

function var_dot = ClosedLoop_QuadrotorEOM_Outer_Nonlinear(t, var, g, m, I, d, km, nu, mu)
[Fc, Gc] = VelocityReferenceFeedback(t, var);
force_moment_mat = [Fc(3); Gc]; 
calc_mat = [    -1          -1       -1        -1     ;
            -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2) ;
             d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2);
                km          -km      km        -km  ] ;
          
motor_forces = calc_mat \ force_moment_mat;

var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end

%% 3.6 func
function [Fc, Gc] = VelocityReferenceFeedback(t, var)
% calculate the control forces and moments to achieve a 1.0 m inertial displacement in 2.0 sec

m = 0.068;
g = 9.81;

phi = var(4); 
theta = var(5); 
psi = var(6);
u = var(7); 
v = var(8); 
w = var(9);
p = var(10); 
q = var(11); 
r = var(12);

K1_lat  = 0.0013;
K1_long = 0.0016;
K1_spin = 0.0040;
K2_lat  = 0.0023;
K2_long = 0.0029;
K2_psi  = 0;
K3_lat_opt  = 1.121121121121121e-04;
K3_long_opt = -1.121121121121121e-04;

% 1.0 m displacement in 2.0 sec requires a constant 0.5 m/s velocity
if t >= 0 && t <= 2.0
    u_ref = 0.5;
    v_ref = 0.5;
else
    u_ref = 0;
    v_ref = 0;
end

% Fc and GC calc
Fc = [0; 0; -m*g];

L = -K1_lat*p  - K2_lat*phi   - K3_lat_opt*(v - v_ref);
M = -K1_long*q - K2_long*theta - K3_long_opt*(u - u_ref);
N = -K1_spin*r - K2_psi*psi;

Gc = [L; M; N];

end

%% Old func
function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
    ignore_drag = 0;
    phi = var(4); 
    theta = var(5); 
    psi = var(6);
    u = var(7); 
    v = var(8); 
    w = var(9);
    p = var(10); 
    q = var(11); 
    r = var(12);
    R = eul2rotm([psi, theta, phi], "ZYX");
    x_1_dot = R * [u; v; w];
    x_2_dot = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)*sec(theta) cos(phi)*sec(theta)] * [p; q; r];
    A_mat = [-1 -1 -1 -1; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2); d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2); km -km km -km];
    cont_vec = A_mat * motor_forces;
    w_skew = [0 -r q; r 0 -p; -q p 0] ;
    x_3_dot = -w_skew * [u; v; w] - g * [sin(theta); -cos(theta)*sin(phi); -cos(theta)*cos(phi)] + (1/m)*[0; 0; cont_vec(1)] - (nu/m)*[u; v; w];
    x_4_dot = inv(I) * ([cont_vec(2); cont_vec(3); cont_vec(4)] - w_skew * I * [p; q; r] - mu * [p; q; r]);
    if (ignore_drag == 1)
    x_3_dot = -w_skew * [u; v; w] - g * [sin(theta); -cos(theta)*sin(phi); -cos(theta)*cos(phi)] + (1/m)*[0; 0; cont_vec(1)];
    x_4_dot = inv(I) * ([cont_vec(2); cont_vec(3); cont_vec(4)] - w_skew * I * [p; q; r]);
    end
    var_dot = [x_1_dot; x_2_dot; x_3_dot; x_4_dot];
end

