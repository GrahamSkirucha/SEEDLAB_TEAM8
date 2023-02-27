
% Required file: motordata.csv, motrTF.slx
% This program imports real data from a csv and then plots the data along
% with the calculated transfer function model output 
%

k = 0.823; % Proportional K term
r = 14.93; % sigma term
A = readtable("motordata.csv"); % open csv of collected motor data
A = table2array(A); % turn the table into an array

file = 'motorTf';
open_system(file) % open the simulink file for the transfer fn of motor
out=sim(file);% get the simulink output
B=A; % create a copy of the data

i = size(A); % get size of array
i = i(1); % get the number of rows
for j = 2:i % for every element in the array
    B(j,2) = (A(j,2)-A(j-1,2))/(A(j,1)-A(j-1,1)); %find dtheta/dt (velocity)
end

C=B; % create an array to show the constant final value

for j = 1:i
     B(j,1) = B(j,1)/1000; %convert from indecies to seconds 
     C(j,2) = k; % make C a constant value of k
end 

hold on
plot(out.tout,out.TF) % plot the motor transfer function model data
plot(C(:,1),C(:,2)) % plot the constant final value
plot(B(:,1),B(:,2)); % plot the real motor step response data
axis([.900,2.000,0,1]) % scale axis
% set axis titles
xlabel("Time (s)")
ylabel("Velocity (counts/ms)")
title("Real and modeled motor step response")
legend("Simulated Data","Final Value","Experimental Data")
hold off
