clear
clc
close all

%% variables
g = 9.81;

lambda1 = -2; % rad/s (-1/0.5s)
lambda2 = -20; % rad/s (dominate lambda1)

K2_I = (lambda1 * lambda2);
K1_I = -(lambda1 + lambda2);

%{
Lc = -K1p - K2phi + K3(Vref - V)
vdot = gphi
phidot = p
pdot = -K2_Iphi + K1_Ip + K2_Iphi(K3(Vref - V))

A_lat = [0,         g,      0; 
         0,         0,      1; 
     -K2_I*K3,   -K2_I,   -K1_I];

A_long = [0,         -g,      0;    note negative g
         0,         0,      1; 
     -K2_I*K3,   -K2_I,   -K1_I];

K3lat = -K3long

max time constant = 1.25 sec -> -0.8 rad
%}

%% Root Locus
% run K3 from 0 to 0.1 and gather eigenvalues
K3_test = linspace(0, 0.1, 1000);
eigs_lat = zeros(3, length(K3_test));

is_real = zeros(1, length(K3_test)); % used later to find most optimal k3

for i = 1:length(K3_test)
    K3 = K3_test(i);
    
    A_lat = [0,         g,      0; 
             0,         0,      1; 
         -K2_I*K3,   -K2_I,   -K1_I];
         
    % calculate eigenvalues
    eigs_lat(:, i) = eig(A_lat);
    is_real(i) = isreal(eigs_lat(:, i));
end

%% find best K3

% find range of valid k3 (time const (-0.8) to max)
valid_idx = find(is_real & max(real(eigs_lat)) <= -0.8);
K3_min = K3_test(valid_idx(1));
K3_max = K3_test(valid_idx(end));

% find poles and time const and K3(max)
opt_poles = eigs_lat(:, valid_idx(end)); % pull eigenvalues

dom_pole = max(real(opt_poles)); % dominant is largest pole

tau = -1 / dom_pole; % time constant

K3_lat_opt = K3_max; % pull k3 value with index
K3_long_opt = -K3_max; % long calc

%% Locus plot
%{
good poles are the tiny horizontal line between the max time constant(0.8)
and when the pole locuses split 0.96-98. This translates to a K3 range of
0.0470 to 0.0484 with curr vals. Optimal is the max at 0.0484
%}

figure()
hold on
grid on

% plot root locus
plot(real(eigs_lat(1,:)), imag(eigs_lat(1,:)), 'b', 'DisplayName', 'Pole 1 Locus');
plot(real(eigs_lat(2,:)), imag(eigs_lat(2,:)), 'r--', 'DisplayName', 'Pole 2 Locus');
plot(real(eigs_lat(3,:)), imag(eigs_lat(3,:)), 'g--', 'DisplayName', 'Pole 3 Locus');
% plot max time constant
xline(-0.8, 'k','LineWidth',1.5, 'DisplayName','Max Time Constant');


title('Root Locus Varying K3');
xlabel('Real Axis (rad/s)');
ylabel('Imaginary Axis (rad/s)');
legend('Location', 'best');
hold off;