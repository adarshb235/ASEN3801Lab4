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
v_aero = 1e-3; 

% Aerodynamic moment coefficient (N*m/(rad/s)^2)
mu_aero = 2e-6;

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



