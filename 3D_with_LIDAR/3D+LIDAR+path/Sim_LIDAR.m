function Sim_LIDAR

global A
% 
A.X1 = A.X + A.LIDAR_X_offset;
A.Y1 = A.Y + A.LIDAR_Y_offset;
A.Z1 = A.Z + A.LIDAR_Z_offset;

% A.X1 = A.X;
% A.Y1 = A.Y;
% A.Z1 = A.Z;

tic

[A.X2,A.Y2,A.Z2]=rotateXYZ(A.X2_init,A.Y2_init,A.Z2_init,A.phi,A.theta,A.psi);
A.Z2 = A.Z2+A.Z1;
A.X2 = A.X2+A.X1;
A.Y2 = A.Y2+A.Y1;

Beam_distances = 4*ones(1,181);
Beam_distances_ground = 4*ones(1,181);
A.LIDARBeams = 4*ones(1,181);
%--------------- Obstacles --------------------%
for i=1:A.num_obstacles(1)

m = -1*(A.M1(i)*(A.X1*ones(1,181)-A.D1(i,1)) - A.M2(i)*(A.Y1*ones(1,181)-A.D1(i,2)) + A.M3(i)*(A.Z1*ones(1,181)-A.D1(i,3)))./(A.M1(i)*(A.X2-A.X1) - A.M2(i)*(A.Y2-A.Y1) + A.M3(i)*(A.Z2-A.Z1));

X_suggested = (A.X1 + (A.X2 - A.X1).*m);
Y_suggested = (A.Y1 + (A.Y2 - A.Y1).*m);
Z_suggested = (A.Z1 + (A.Z2 - A.Z1).*m);

% check if the intersection is in the same direction of them beam or not
BI1 = (sign(A.X2-A.X1)==sign(X_suggested-A.X1));
BI2 = (sign(A.Y2-A.Y1)==sign(Y_suggested-A.Y1));
BI3 = (sign(A.Z2-A.Z1)==sign(Z_suggested-A.Z1));

% intersection with the obstacle
BI5 = (X_suggested>=A.B1(i,1)-eps*10);
BI6 = (X_suggested<=A.C1(i,1)+eps*10);
BI7 = (Y_suggested>=A.B1(i,2)-eps*10);
BI8 = (Y_suggested<=A.C1(i,2)+eps*10);
BI9 = (Z_suggested>=A.B1(i,3)-eps*10);
BI10 = (Z_suggested<=A.D1(i,3)+eps*10);

BI = ((BI1&BI2&BI3)&BI5&BI6&BI7&BI8&BI9&BI10);
Beam_distances(BI) = sqrt((X_suggested(BI)-A.X1).^2 + (Y_suggested(BI)-A.Y1).^2 + (Z_suggested(BI)-A.Z1).^2);
A.LIDARBeams = min(A.LIDARBeams,Beam_distances);
end
%------------------ Ground ------------------------%
m = -1*A.Z1*ones(1,181)./((A.Z2-A.Z1));

X_suggested = (A.X1 + (A.X2 - A.X1).*m);
Y_suggested = (A.Y1 + (A.Y2 - A.Y1).*m);
Z_suggested = (A.Z1 + (A.Z2 - A.Z1).*m);

BI1 = (sign(A.X2-A.X1)==sign(X_suggested-A.X1));
BI2 = (sign(A.Y2-A.Y1)==sign(Y_suggested-A.Y1));
BI3 = (sign(A.Z2-A.Z1)==sign(Z_suggested-A.Z1));
BI = (BI1&BI2&BI3);

Beam_distances_ground(BI) = sqrt((X_suggested(BI)-A.X1).^2 + (Y_suggested(BI)-A.Y1).^2 + (Z_suggested(BI)-A.Z1).^2);

% A.LIDARBeams = min(A.LIDARBeams,Beam_distances);
A.LIDARBeams = min(A.LIDARBeams,Beam_distances_ground);
end