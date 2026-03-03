function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig, col)
%% inertial posiition (rows 1-3)
figure(fig(1));
subplot(311); %xe
plot(time, aircraft_state_array(1,:), col); hold on;
subplot(312);%ye
plot(time, aircraft_state_array(2,:), col); hold on;
subplot(313);%ze
plot(time, aircraft_state_array(3,:), col); hold on;
%% euler angles (rows 4-6)
figure(fig(2));
subplot(311);%phi
plot(time, aircraft_state_array(4,:), col); hold on;
subplot(312);%theta
plot(time, aircraft_state_array(5,:), col); hold on;
subplot(313);%psi
plot(time, aircraft_state_array(6,:), col); hold on;
%% inertial velocity in B frame (rows 7-9)
figure(fig(3));
subplot(311);%ue
plot(time, aircraft_state_array(7,:), col); hold on;
subplot(312);%ve
plot(time, aircraft_state_array(8,:), col); hold on;
subplot(313);%we
plot(time, aircraft_state_array(9,:), col); hold on;
%% Angular rates (rows 10-12)
figure(fig(4));
subplot(311);%p
plot(time, aircraft_state_array(10,:), col); hold on;
subplot(312);%q
plot(time, aircraft_state_array(11,:), col); hold on;
subplot(313);%r
plot(time, aircraft_state_array(12,:), col); hold on;
%% Control input Variable 
figure(fig(5));
subplot(311); %Zc
plot(time, control_input_array(1,:), col); hold on;
subplot(312);%Lc
plot(time, control_input_array(2,:), col); hold on;
subplot(313);%Mc
plot(time, control_input_array(3,:), col); hold on;
subplot(314);%Nc
plot(time, control_input_array(4,:), col); hold on;
%% Plot of 3D path
figure(fig(6));
plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),aircraft_state_array(3,:), col); hold on;
end

