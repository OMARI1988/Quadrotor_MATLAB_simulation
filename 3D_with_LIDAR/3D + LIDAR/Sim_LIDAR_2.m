function Sim_LIDAR_2

global A

A.X1 = A.X;
A.Y1 = A.Y;
A.Z1 = A.Z;

tic

[A.X2,A.Y2,A.Z2]=rotateXYZ(A.X2_init,A.Y2_init,A.Z2_init,A.theta,A.phi,A.psi);
A.Z2 = A.Z2+A.Z1;
A.X2 = A.X2+A.X1;
A.Y2 = A.Y2+A.Y1;

Beam_distances = 4*ones(1,181);
Beam_distances_ground = 4*ones(1,181);
A.LIDARBeams = 4*ones(1,181);
%--------------- Obstacles --------------------
m = -1*(A.M1*(A.X1*ones(1,181)-A.D1(1)) - A.M2*(A.Y1*ones(1,181)-A.D1(2)) + A.M3*(A.Z1*ones(1,181)-A.D1(3)))./(A.M1*(A.X2-A.X1) - A.M2*(A.Y2-A.Y1) + A.M3*(A.Z2-A.Z1));

X_suggested = (A.X1 + (A.X2 - A.X1).*m);
Y_suggested = (A.Y1 + (A.Y2 - A.Y1).*m);
Z_suggested = (A.Z1 + (A.Z2 - A.Z1).*m);

BI1 = (sign(A.X2-A.X1)==sign(X_suggested-A.X1));
BI2 = (sign(A.Y2-A.Y1)==sign(Y_suggested-A.Y1));
BI3 = (sign(A.Z2-A.Z1)==sign(Z_suggested-A.Z1));

BI5 = (X_suggested>=-1);
BI6 = (X_suggested<=1);
BI7 = (Y_suggested>=0);
BI8 = (Y_suggested<=2);
BI9 = (Z_suggested>=0);
BI10 = (Z_suggested<=(Y_suggested/2));

BI = ((BI1&BI2&BI3)&BI5&BI6&BI7&BI8&BI9&BI10);
Beam_distances(BI) = sqrt((X_suggested(BI)-A.X1).^2 + (Y_suggested(BI)-A.Y1).^2 + (Z_suggested(BI)-A.Z1).^2);


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

A.LIDARBeams = min(A.LIDARBeams,Beam_distances);
A.LIDARBeams = min(A.LIDARBeams,Beam_distances_ground);
end