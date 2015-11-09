function SimInitial2
global A
% 
% A.X1wall = 0;          % X first point in wall     NF were X is on the right direction and Y is in front !!
% A.Y1wall = 1;           % Y first point in wall
% A.X2wall = .5;           % X second point in wall   X2 has to be > X1
% A.Y2wall = 1;           % Y second point in wall
% 
% A.Z1wall = 0;           % Z lower edge
% A.Z2wall = 1;           % Z higher edge

% D1 B1 C1 are three points on the wall that form a plane [X,Y,Z]
% the ordering of the points will be like this
% B1 = [X1,Y1,Z1]
% C1 = [X2,Y2,Z1]
% D1 = [(X1+X2)/2,(Y1+Y2)/2,Z2]

% samll room obstacle 
Xi = [-1.5 -1.5 .5 -1.5 -1.5 -1.5 3 1.5 1.5];
Xf = [1.5 -.5 1.5 1.5 -1.5 3 3 3 1.5];
Yi = [1 1 1 1 1 5 2 2 1];
Yf = [1 1 1 1 5 5 5 2 2];
Zi = [0 .5 .5 1 0 0 0 0 0];
Zf = [.5 1 1 1.5 1.5 1.5 1.5 1.5 1.5];


% random wall 
Xi = [-1.5 -3 -1];
Xf = [1 -2.5 1.5];
Yi = [1 -.5 2.2];
Yf = [1 2 2.2];
Zi = [0 0 0];
Zf = [1.5 2.5 3];

for i=1:length(Xi)
[A.B1(i,:),A.C1(i,:),A.D1(i,:)] = obstacle([Xi(i) Xf(i)],[Yi(i) Yf(i)],[Zi(i) Zf(i)]);
% [A.B1(2,:),A.C1(2,:),A.D1(2,:)] = obstacle([-1.5 -.5],[1 1],[.5 1]);
end

% A.B1(1,:) = [-1,1,0];
% A.C1(1,:) = [1,1,0];
% A.D1(1,:) = [.5,1,1];

%obstacle num 2
% A.B1(2,:) = [0,2,0];
% A.C1(2,:) = [2,2,0];
% A.D1(2,:) = [1,2,2];

%obstacle num 3
% A.B1(3,:) = [-2,1,0];
% A.C1(3,:) = [-1,3,0];
% A.D1(3,:) = [-1.5,2,1.5];

A.num_obstacles = size(A.D1);

for i=1:A.num_obstacles(1)
    
patch([A.B1(i,1) A.C1(i,1) A.C1(i,1) A.B1(i,1)],[A.B1(i,2) A.C1(i,2) A.C1(i,2) A.B1(i,2)],[A.B1(i,3) A.B1(i,3) A.D1(i,3) A.D1(i,3)],[0 .4 1])
    
DB = A.B1(i,:)-A.D1(i,:);
DC = A.C1(i,:)-A.D1(i,:);

A.M1(i) = DB(2)*DC(3) - DB(3)*DC(2);
A.M2(i) = DB(1)*DC(3) - DB(3)*DC(1);
A.M3(i) = DB(1)*DC(2) - DB(2)*DC(1);
end

A.X1 = 0;                % quadrotors position X
A.Y1 = 0;                % quadrotors position Y
A.Z1 = .5;               % quadrotors position Z

A.theta11 = [0:pi/180:pi];  % used in ploting the LIDAR beams
end