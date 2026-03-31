clear
clc
close all

%% Variable Declaration

% mass (kg)
m = 0.068; 

% radial distance from CG to propeller (m)
d = 0.060; 

% Control moment coefficient (N*m/N)
km = 0.0024; 

% Moments of Inertia (kg*m^2)
Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1.0e-4;

I = diag([Ix, Iy, Iz]);

% Aerodynamic force coefficient (N/(m/s)^2)
nu = 1e-3; 

% Aerodynamic moment coefficient (N*m/(rad/s)^2)
mu = 2e-6;

% gravity constant (m/s^2)
g = 9.81;

% 12x1 column vector of state deviations
%     [x; y; z; phi; theta; psi; u; v; w; p; q; r]
var = [0; 0; 0; 0; 0; 0; 0; 0; 0; 100; 100; 100];

%% 3.1
%{
s^2 + (K1/I)s + (K2/I) = 0

(s-lambda1)(s-lambda2) = 0
s^2 - (lambda1 + lambda2)s + (lambda1*lambda2) = 0

K1/I = -(lambda1 + lambda2) -> K1 = -I(lambda1 + lambda2)

K2/I = (lambda1*lambda2) -> K2 = I(lambda1*lambda2)
%}
% eigenvalues
lambda1 = -2; % rad/s (-1/0.5s)
lambda2 = -20; % rad/s (dominate lambda1)

% Lateral gains (roll x)
K2_lat = Ix*(lambda1 * lambda2);
K1_lat = -Ix*(lambda1 + lambda2);

% Longitudinal gains (pitch y)
K2_long = Iy*(lambda1 * lambda2);
K1_long = -Iy*(lambda1 + lambda2);

% spin (yaw z)
K1_spin = 0.004;
K2_psi = 0;


%% 3.3 main code
tspan = [0 10]; % 10 sec simulation window

% a. Deviation by +5 deg in roll
var0_a = zeros(12, 1);
var0_a(4) = deg2rad(5);
[t_a, var_a] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Linearized(t, var, g, m, I), tspan, var0_a);
% a plot
figure()
plot(t_a, rad2deg(var_a(:,4)))
hold on
grid on
title('a. Deviation by +5 deg in roll')
xlabel('Time (s)')
ylabel('Roll Angle (deg)')
hold off

% b. Deviation by +5 deg in pitch
var0_b = zeros(12, 1);
var0_b(5) = deg2rad(5);
[t_b, var_b] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Linearized(t, var, g, m, I), tspan, var0_b);
% b plot
figure()
plot(t_b, rad2deg(var_b(:,5)))
hold on
grid on
title('b. Deviation by +5 deg in pitch')
xlabel('Time (s)')
ylabel('Pitch Angle (deg)')
hold off

% c. Deviation by +0.1 rad/sec in roll rate
var0_c = zeros(12, 1);
var0_c(10) = 0.1;
[t_c, var_c] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Linearized(t, var, g, m, I), tspan, var0_c);
% c plot
figure()
plot(t_c, var_c(:,10))
hold on
grid on
title('c. Deviation by +0.1 rad/sec in roll rate')
xlabel('Time (s)'); ylabel('Roll Rate (rad/s)')
hold off

% d. Deviation by +0.1 rad/sec in pitch rate
var0_d = zeros(12, 1);
var0_d(11) = 0.1;
[t_d, var_d] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Linearized(t, var, g, m, I), tspan, var0_d);
% d plot
figure()
plot(t_d, var_d(:,11))
hold on
grid on
title('d. Deviation by +0.1 rad/sec in pitch rate')
xlabel('Time (s)')
ylabel('Pitch Rate (rad/s)')
hold off

%% 3.4 main code
%{
These graphs should be very similar to the 3.3 linear oem graphs since our
deviations are relatively small so sin(theta) = theta applies
%}
% a. Deviation by +5 deg in roll
var0_a = zeros(12, 1);
var0_a(4) = deg2rad(5);
[t_a, var_a] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Nonlinear(t, var, g, m, I, d, km, nu, mu), tspan, var0_a);
% a plot
figure()
plot(t_a, rad2deg(var_a(:,4)))
hold on
grid on
title('a. Nonlinear Deviation by +5 deg in roll')
xlabel('Time (s)')
ylabel('Roll Angle (deg)')
hold off

% b. Deviation by +5 deg in pitch
var0_b = zeros(12, 1);
var0_b(5) = deg2rad(5);
[t_b, var_b] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Nonlinear(t, var, g, m, I, d, km, nu, mu), tspan, var0_b);
% b plot
figure()
plot(t_b, rad2deg(var_b(:,5)))
hold on
grid on
title('b. Nonlinear Deviation by +5 deg in pitch')
xlabel('Time (s)')
ylabel('Pitch Angle (deg)')
hold off

% c. Deviation by +0.1 rad/sec in roll rate
var0_c = zeros(12, 1);
var0_c(10) = 0.1;
[t_c, var_c] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Nonlinear(t, var, g, m, I, d, km, nu, mu), tspan, var0_c);
% c plot
figure()
plot(t_c, var_c(:,10))
hold on
grid on
title('c. Nonlinear Deviation by +0.1 rad/sec in roll rate')
xlabel('Time (s)'); ylabel('Roll Rate (rad/s)')
hold off

% d. Deviation by +0.1 rad/sec in pitch rate
var0_d = zeros(12, 1);
var0_d(11) = 0.1;
[t_d, var_d] = ode45(@(t, var) ClosedLoop_QuadrotorEOM_Nonlinear(t, var, g, m, I, d, km, nu, mu), tspan, var0_d);
% d plot
figure()
plot(t_d, var_d(:,11))
hold on
grid on
title('d. Nonlinear Deviation by +0.1 rad/sec in pitch rate')
xlabel('Time (s)')
ylabel('Pitch Rate (rad/s)')
hold off


%% 3.2 
%{
Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
aircraft state var. The control force in the body 𝑧-direction should still equal the weight of the
quadrotor (hard code the values in the function along with the control gains). Set the control moment
using the control laws from Problem 3.1.
%}

function [Fc, Gc] = InnerLoopFeedback(var)
% var = input state vector
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc]

% eigenvalues
lambda1 = -2; % rad/s (-1/0.5s)
lambda2 = -20; % rad/s (dominate lambda1)

Ix = 5.8e-5;
Iy = 7.2e-5;

% Lateral gains (roll x)
K2_lat = Ix*(lambda1 * lambda2);
K1_lat = -Ix*(lambda1 + lambda2);

% Longitudinal gains (pitch y)
K2_long = Iy*(lambda1 * lambda2);
K1_long = -Iy*(lambda1 + lambda2);

% spin (yaw z)
K1_spin = 0.004;

% mass and g
m = 0.068; 
g = 9.81;

phi = var(4);  % roll angle
theta = var(5);  % pitch angle

p = var(10); % roll rate
q = var(11); % pitch rate
r = var(12); % yaw rate

% Fc = [Xc, Yc, Zc]
Fc = [0; 0; m*g]; % equal the weight of the quadrotor 

% Gc = [Lc, Mc, Nc]  Lec12-3728_final.pdf slide 8 
Lc = -K2_lat * phi - K1_lat * p; % roll moment
Mc = -K2_long * theta - K1_long * q; % pitch moment
Nc = -K1_spin * r; % yaw moment

Gc = [Lc; Mc; Nc];

end

%% 3.3
%{
Using the function from Problem 3.2, create a new EOM function to simulate the response of the
closed loop linearized system. Use this new function to simulate initial condition deviations from
the steady hover trim state as follows (use a 10 sec simulation window):
a. Deviation by +5 deg in roll
b. Deviation by +5 deg in pitch
c. Deviation by +0.1 rad/sec in roll rate
d. Deviation by +0.1 rad/sec in pitch rate
%}


function var_dot = ClosedLoop_QuadrotorEOM_Linearized(t, var, g, m, I)
% t = time, var =  delta input state vector, m = mass, g = gravity constant
% deltaFc and deltaGc are deviations from the steady hover trim condition
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc]
% var =     [dx; dy; dz; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr]

var_dot = zeros(12, 1);

% calculate delta Fc and Gc from func
[Fc, Gc] = InnerLoopFeedback(var);
deltaFc = Fc - [0; 0; m*g]; 
deltaGc = Gc - [0; 0; 0];

% delta pos
var_dot(1:3) = var(7:9); % xdot, ydot, zdot = u, v ,w

% delta euler
var_dot(4:6) = var(10:12); %phidot, thetadot, psidot = p, q, r

% delta vel
var_dot(7:9) = g .* [-var(5); var(4); 0] + 1/m .* [0; 0; deltaFc(3)];

% delta angular vel
Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

var_dot(10:12) = [1/Ix * deltaGc(1); 1/Iy * deltaGc(2); 1/Iz * deltaGc(3)];

end

%% 3.4
%{
Repeat Problem 3.3 (ie. create a new EOM function that adds the control to the nonlinear quadrotor
dynamics) using the nonlinear dynamics model together with the feedback control design for the
linearized system, and compare the closed loop linearized and nonlinear behaviors.
%}

function var_dot = ClosedLoop_QuadrotorEOM_Nonlinear(t, var, g, m, I, d, km, nu, mu)
% t = time, var =  delta input state vector, m = mass, g = gravity constant
% deltaFc and deltaGc are deviations from the steady hover trim condition
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc]
% var =     [dx; dy; dz; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr]

[Fc, Gc] = InnerLoopFeedback(var);

force_moment_mat = [-Fc(3); Gc];

calc_mat = [    -1          -1       -1        -1     ;
            -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2) ;
             d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2);
                km          -km      km        -km  ] ;

          
motor_forces = calc_mat \ force_moment_mat;

var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end

% non linearized eom func for 3.4
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

