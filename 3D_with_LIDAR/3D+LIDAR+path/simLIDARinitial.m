function simLIDARinitial
global A

% A.LIDARResolution=1;
% A.LIDARMaxRange=4;
% A.LIDARNumberOfBeams=1+180/A.LIDARResolution;
% A.LIDARBeamAngles = (0:A.LIDARResolution*pi/180:pi)';
% A.LIDARBeamSlopes = tan(A.LIDARBeamAngles);

A.LIDAR_X_offset = 0;
A.LIDAR_Y_offset = 0;
A.LIDAR_Z_offset = 0;


A.LIDAR_X = [A.X1 cos(A.theta11)] + A.LIDAR_X_offset;
A.LIDAR_Y = [A.Y1 sin(A.theta11)] + A.LIDAR_Y_offset;
A.LIDAR_Z = [A.Z1 zeros(1,181)] + A.LIDAR_Z_offset;
A.Cdata = ones(1,182);

A.LIDAR_Patch = patch(A.LIDAR_X,A.LIDAR_Y,A.LIDAR_Z,A.Cdata,'facecolor','r','FaceAlpha',.3);


A.LIDARBeams = 4*ones(1,181);

A.t=0:180;

A.X2_init = 4*cosd(A.t);
A.Y2_init = 4*sind(A.t);
A.Z2_init = zeros(1,181);