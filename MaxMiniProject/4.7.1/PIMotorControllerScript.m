
% Required file: PIMotorController.slx motordataclosed.csv
% This program plots the step response of the motor PI controller 
% P value = 0.267364131777782
% I value = 10.8690458245462

theta = 1; % desired final angle
k = 0.823; % k value for motor transfer fn
r = 14.93; % sigma value for motor transfer fn

A = readtable("motordataclosed.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array
i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 1:i
     A(j,1) = A(j,1)/1000; %convert from indecies to seconds 
     A(j,2) = A(j,2)/360; %convert from degrees to revolutions
end 
% open simulation
%
open_system('PIMotorController')

% run the simulation
%
out=sim('PIMotorController');

%plot data
%
figure
hold on
plot(out.simout)
plot(A(:,1),A(:,2))
ylabel("Position (revolutions)")
xlabel("Time (s)")
legend("Simulation Data","Experiment Data")
