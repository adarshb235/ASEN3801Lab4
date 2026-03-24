clc;
clear;
close all;



m = 0.068; % kg
d = 0.06; % m
km = 0.0024; % Nm/N
Ix = 5.8e-5; % kg m^2
Iy = 7.2e-5; % kg m^2
Iz = 1e-4; % kg m^2
g = 9.81; % m/s^2
nu = 1e-3; % N/(m/s)^2
mu = 2e-6; % Nm/(rad/s)^2
I = [Ix 0 0; 0 Iy 0; 0 0 Iz]; % Inertia matrix
f_trim = m * g / 4;
motor_forces = [f_trim; f_trim; f_trim; f_trim];
t_span = [0 10];
var_init_hover = zeros(12, 1);
[t_hover, var_hover] = ode45(@(t_hover, var_hover) QuadrotorEOM(t_hover, var_hover, g, m, I, d, km, nu, mu, motor_forces), t_span, var_init_hover);
A_mat = [-1 -1 -1 -1; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2); d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2); km -km km -km];
cont_vec_hover = A_mat * motor_forces;
var_hover(abs(var_hover) < 1e-10) = 0;

PlotAircraftSim(t_hover, var_hover', cont_vec_hover, [1 2 3 4 5 6], 'b-');   

v_trim_V5 = 5;
phi_trim_V5 = asin((nu * v_trim_V5) / (m * g));
theta_trim_V5 = 0;
psi_trim_V5 = 0;
var_init_V5 = zeros(12, 1);
var_init_V5(4) = phi_trim_V5;                
var_init_V5(8) = 5 * cos(phi_trim_V5);        
var_init_V5(9) = -5 * sin(phi_trim_V5);  
f_trim_V5 = m * g / cos(phi_trim_V5)/ 4;
motor_forces_V5 = [f_trim_V5; f_trim_V5; f_trim_V5; f_trim_V5];
cont_vec_V5 = A_mat * motor_forces_V5;
[t_V5, var_V5] = ode45(@(t_V5, var_V5) QuadrotorEOM(t_V5, var_V5, g, m, I, d, km, nu, mu, motor_forces_V5), t_span, var_init_V5);
var_V5(abs(var_V5) < 1e-10) = 0;

PlotAircraftSim(t_V5, var_V5', cont_vec_V5, [1 2 3 4 5 6], 'r-'); 

psi_trim_U5 = pi/2;            
u_trim_U5 = 5;                  
theta_trim_U5 = asin((nu * u_trim_U5) / (m * g)); 
phi_trim_U5 = 0;                

f_trim_U5 = (m * g * cos(theta_trim_U5)) / 4;
motor_forces_U5 = [f_trim_U5; f_trim_U5; f_trim_U5; f_trim_U5];

var_init_U5 = [0; 0; 0; phi_trim_U5; theta_trim_U5; psi_trim_U5; u_trim_U5; 0; 0; 0; 0; 0];

[t_U5, var_U5] = ode45(@(t_U5, var_U5) QuadrotorEOM(t_U5, var_U5, g, m, I, d, km, nu, mu, motor_forces_U5), t_span, var_init_U5);
var_U5(abs(var_U5) < 1e-10) = 0;  
cont_vec_U5 = A_mat * motor_forces_U5;

PlotAircraftSim(t_U5, var_U5', cont_vec_U5, [1 2 3 4 5 6], 'g-'); 

trim_titles = {'Inertial Position', 'Euler Angles', 'Body Frame Velocities', 'Angular Rates', '3D Trajectory'};
for f = 1:5
    figure(f);
    sgtitle('Prob 1.2 & 1.4: Trim State Validations', 'FontSize', 14, 'FontWeight', 'bold');
    if f < 5
        subplot(3,1,1);
        legend('Hover', '5 m/s Lateral', '5 m/s Forward', 'Location', 'best');
    end
end

deviations = zeros(6, 12);
deviations(1, 4)  = 5 * pi/180;    % a) +5 deg roll
deviations(2, 5)  = 5 * pi/180;    % b) +5 deg pitch
deviations(3, 6)  = 5 * pi/180;    % c) +5 deg yaw
deviations(4, 10) = 0.1;           % d) +0.1 rad/s roll rate
deviations(5, 11) = 0.1;           % e) +0.1 rad/s pitch rate
deviations(6, 12) = 0.1;           % f) +0.1 rad/s yaw rate

titles = {'Roll', 'Pitch', 'Yaw', 'Roll Rate (p)', 'Pitch Rate (q)', 'Yaw Rate (r)'};
dev_amounts = {'+5 deg', '+5 deg', '+5 deg', '+0.1 rad/s', '+0.1 rad/s', '+0.1 rad/s'};

for i = 1:6
    var_init = zeros(12, 1) + deviations(i, :)';
    [t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), [0 10], var_init);
    fig_offsets = 10 + (i-1)*6 + [1:6]; 
    PlotAircraftSim(t, var', cont_vec_hover, fig_offsets, 'b-');
    for f = 1:5
        figure(fig_offsets(f));
        sgtitle(sprintf('Prob 2.1: Uncontrolled Deviation %s in %s', dev_amounts{i}, titles{i}), 'FontSize', 14, 'FontWeight', 'bold');  
    end
end



rate_dev_indices = [10, 11, 12];
rate_dev_names = {'Roll Rate (p)', 'Pitch Rate (q)', 'Yaw Rate (r)'};

for i = 1:3    
    var_init = zeros(12, 1);
    var_init(rate_dev_indices(i)) = 0.1; 
    
    % Simulate Uncontrolled System
    [t_un, var_un] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), t_span, var_init);
    thrusts_un = repmat(motor_forces, 1, length(t_un));
    cont_vec_un = A_mat * thrusts_un;
    
    % Simulate Controlled System
    [t_c, var_c] = ode45(@(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), t_span, var_init);
    thrusts_c = zeros(4, length(t_c));
    cont_vec_c = zeros(4, length(t_c));
    for k = 1:length(t_c)
        state_k = var_c(k, :)';
        [Fc_k, Gc_k] = RotationDerivativeFeedback(state_k, m, g);
        thrusts_c(:, k) = ComputeMotorForces(Fc_k, Gc_k, d, km);
        cont_vec_c(:, k) = [Fc_k(3); Gc_k]; 
    end
    
    % 3. Plotting Overlays (Grouped in 110s, 120s, 130s)
    fig_base = 100 + (i * 10); 
    fig_offsets = fig_base + [1:6];
    
    PlotAircraftSim(t_un, var_un', cont_vec_un, fig_offsets, 'r--');
    PlotAircraftSim(t_c, var_c', cont_vec_c, fig_offsets, 'b-');
    
    for f = 1:5
        figure(fig_offsets(f));
        sgtitle(sprintf('Prob 2.5: Controller Overlay (+0.1 rad/s in %s)', rate_dev_names{i}), 'FontSize', 14, 'FontWeight', 'bold');
        subplot(3,1,1); legend('Uncontrolled', 'Controlled', 'Location', 'best');
    end
    
    % Plot Motor Thrusts
    figure(fig_base + 7); hold on; grid on; box on;
    sgtitle(sprintf('Prob 2.5: Motor Thrusts (+0.1 rad/s in %s)', rate_dev_names{i}), 'FontSize', 14, 'FontWeight', 'bold');
    for motor = 1:4
        subplot(2, 2, motor); hold on; grid on; box on;
        plot(t_un, thrusts_un(motor, :), 'r--', 'LineWidth', 1.5);
        plot(t_c, thrusts_c(motor, :), 'b-', 'LineWidth', 1.5);
        title(sprintf('Motor %d', motor));
        xlabel('Time (s)'); ylabel('Thrust (N)');
        if motor == 1
            legend('Uncontrolled', 'Controlled', 'Location', 'best');
        end
    end
end


function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
    figure(fig(1));
    hold on;
    subplot(3, 1, 1)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Inertial X Position (m)")
    plot(time, aircraft_state_array(1,:), col);
    subplot(3, 1, 2)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Inertial Y Position (m)")
    plot(time, aircraft_state_array(2,:), col);
    subplot(3, 1, 3)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Inertial Z Position (m)")
    plot(time, aircraft_state_array(3,:), col);

    figure(fig(2));
    hold on;
    subplot(3, 1, 1)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Roll Angle (rad)")
    plot(time, aircraft_state_array(4,:), col);
    subplot(3, 1, 2)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Pitch Angle (rad)")
    plot(time, aircraft_state_array(5,:), col);
    subplot(3, 1, 3)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Yaw Angle (rad)")
    plot(time, aircraft_state_array(6,:), col);

    figure(fig(3));
    hold on;
    subplot(3, 1, 1)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Body Frame X Velocity (m/s)")
    plot(time, aircraft_state_array(7,:), col);
    subplot(3, 1, 2)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Body Frame Y Velocity (m/s)")
    plot(time, aircraft_state_array(8,:), col);
    subplot(3, 1, 3)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Body Frame Z Velocity (m/s)")
    plot(time, aircraft_state_array(9,:), col);

    figure(fig(4));
    hold on;
    subplot(3, 1, 1)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Roll Rate (rad/s)")
    plot(time, aircraft_state_array(10,:), col);
    subplot(3, 1, 2)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Pitch Rate (rad/s)")
    plot(time, aircraft_state_array(11,:), col);
    subplot(3, 1, 3)
    hold on;
    grid on;
    box on;
    xlabel("Time (s)")
    ylabel("Yaw Rate (rad/s)")
    plot(time, aircraft_state_array(12,:), col);

    figure(fig(5));
    hold on;
    grid on;
    plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:));
    plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1), 'go');
    plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end), 'ro');
    view(45, 30)
    axis equal; 
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
end

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

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)    
    var_dot = zeros(12, 1);
    var_dot(1) = var(7);
    var_dot(2) = var(8);
    var_dot(3) = var(9);
    var_dot(4) = var(10);
    var_dot(5) = var(11);
    var_dot(6) = var(12);
    var_dot(7) = -g * var(5);
    var_dot(8) = g * var(4);
    A_mat = [-1 -1 -1 -1; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2); d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2); km -km km -km];
    cont_vec = A_mat * [deltaFc; deltaGc];
    var_dot(9) = cont_vec(1) / m;
    var_dot(10) = cont_vec(2) / I(1,1);
    var_dot(11) = cont_vec(3) / I(2,2);
    var_dot(12) = cont_vec(4) / I(3,3);
end

function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
    Fc = [0 0 m*g];

    p = var(10);
    q = var(11);
    r = var(12);

    Kd = 0.004; 
    L = -Kd * p; 
    M = -Kd * q; 
    N = -Kd * r; 

    Gc = [L; M; N];
end

function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
    A_mat = [-1 -1 -1 -1; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2); d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2); km -km km -km];
    A_mat_inv = A_mat^-1;
    motor_forces = A_mat_inv * [Fc(3); Gc];
end

function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
    ignore_drag = 0;
    d = 0.06; % m
    km = 0.0024; % Nm/N
    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end
