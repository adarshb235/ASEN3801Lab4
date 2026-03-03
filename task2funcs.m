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

%% Test funcs
% [Fc, Gc] = RotationDerivativeFeedback(var, m, g);
% 
% motor_forces = ComputeMotorForces(Fc, Gc, d, km);

%% 2.2 QuadrotorEOM_Linearized
%{
Develop a linearized model of the full quadrotor dynamics about a steady hover trim state. Repeat
Problem 2.1 using the linearized dynamics model, and compare linearized and non-linearized
behaviors on the same plots that you obtained for Problem 2.1 (ie. turn in one final set of plots for the
combined results of Problem 2.1 and 2.2, using the same 10 sec simulation window).
Do not plot responses from multiple deviations on the same plot. For example, don't plot
responses to deviations of +5 deg roll and +5 deg pitch on the same plots, but do plot linearized
and nonlinear responses for +5 deg roll on the same plots).
Your linearized equations of motion function should be of the form:
Where var is the 12x1 column vector of state deviations from the steady hover trim condition,
ordered as presented in the supplementary document. deltaFc and deltaGc are deviations from
the steady hover trim condition
%}

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
% t = time, var =  delta input state vector, m = mass, g = gravity constant
% deltaFc and deltaGc are deviations from the steady hover trim condition
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc]
% var =     [dx; dy; dz; dphi; dtheta; dpsi; du; dv; dw; dp; dq; dr]

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

%% 2.3 RotationDerivativeFeedback
%{
Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
aircraft state var, aircraft mass m, and gravitational acceleration g. The control force in the body 𝑧𝑧-
direction should still equal the weight of the quadrotor. Set the control moments about each body
axis proportional to the rotational rates about their respective axes, but in the opposite sign of the
angular velocity with a gain of 0.004 Nm/(rad/sec):
%}
function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% var = input state vector, m = mass, g = gravity constant
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc]

% control force in the body z direction should equal the weight of the quadrotor
Fc = [0; 0; m*g];

% Set the control moments about each body axis proportional to the 
% rotational rates about their respective axes, but in the opposite sign 
% of the angular velocity with a gain of 0.004 Nm/(rad/sec):

pqr = [var(10); var(11); var(12)]; % rad/s
gain = 0.004; % Nm/(rad/s)

Gc = -pqr .* gain;

end


%% 2.4 ComputeMotorForces
%{
Write a function to calculate the motor thrust forces given the control force and moments:
where motor_forces is the 4 x 1column vector [𝑓𝑓1, 𝑓𝑓2, 𝑓𝑓3, 𝑓𝑓4]𝑇𝑇. The control thrust and moment
vectors are both 3x1 column vectors, while d is the distance from aircraft CG to each rotor and km is
the control moment coefficient.
%}
function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% Fc = [Xc, Yc, Zc], Gc = [Lc, Mc, Nc], d = distance CG to each rotor, km = control moment coefficient

% 4x4
calc_mat = [    -1          -1       -1        -1     ;
            -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2) ;
             d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2);
                km          -km      km        -km  ] ;

% [Zc; Lc; Mc; Nc]
force_moment_mat = [Fc(3); Gc];

% [f1; f2; f3; f4] = 4x4^-1 [Zc; Lc, Mc, Nc]
motor_forces = calc_mat\force_moment_mat; % a\b is inv(a) * b

end

%% 2.5 QuadrotorEOMwithRateFeedback
%{
Add the feedback controller to your nonlinear quadrotor equations of motion. Create a new equation
of motion function with the feedback controller from Problem 2.4 in it:
Repeat simulations for the setups in Problems 2.1.d through 2.1.f. What is the effect of this control
law? Support your argument with new plots for each setup, overlaying the controlled and
uncontrolled nonlinear systems. Also calculate and plot the motor thrust forces for the controlled
and uncontrolled systems.
%}
function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)


end
