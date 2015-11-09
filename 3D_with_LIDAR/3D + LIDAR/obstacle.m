function [B,C,D] = obstacle(X,Y,Z)
% B1 = [X1,Y1,Z1]
% C1 = [X2,Y2,Z1]
% D1 = [(X1+X2)/2,(Y1+Y2)/2,Z2]

B = [X(1),Y(1),Z(1)];
C = [X(2),Y(2),Z(1)];
D = [(X(1)+X(2))/2,(Y(1)+Y(2))/2,Z(2)];


end