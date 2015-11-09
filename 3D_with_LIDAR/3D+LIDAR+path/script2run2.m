
clear all
clc
global A
All_Variables
SimInitial
simLIDARinitial
view(30,30)
grid on
A.Z = .5;

% for i=1:360
%     A.psi =  A.psi +1*pi/180;
% Sim_LIDAR
% LIDAR_Plot
% drawnow
% toc
% end

for i=1:360
    A.phi =  A.phi +1*pi/180;
Sim_LIDAR
LIDAR_Plot
drawnow
toc
end

for i=1:360
    A.theta =  A.theta +1*pi/180;
Sim_LIDAR
LIDAR_Plot
drawnow
toc
end