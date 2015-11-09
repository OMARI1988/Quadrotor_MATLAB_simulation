
global A

% simLIDARinitial;
% A.LIDARBeams = 4*ones(181,1)

A.X22 = A.LIDARBeams.*cos(A.theta11) + A.LIDAR_X_offset;
A.Y22 = A.LIDARBeams.*sin(A.theta11) + A.LIDAR_Y_offset;
A.Z22 = zeros(1,181) + A.LIDAR_Z_offset;
% 
% A.theta = 0*45*pi/180;
% A.phi = 0*45*pi/180;
% A.psi = 45*pi/180;
[A.X22,A.Y22,A.Z22]=rotateXYZ(A.X22,A.Y22,A.Z22,A.phi,A.theta,A.psi);
% Z
set(A.LIDAR_Patch,'xdata',A.X22'+A.X,'ydata',A.Y22'+A.Y,'zdata',A.Z22+A.Z)

% axis([-4 4 -4 4 -4 4])

% grid on
% view(50,50)