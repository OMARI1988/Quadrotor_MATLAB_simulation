global A

% hold on
% if(A.init==0)
%    plot3(A.X_path,A.Y_path,A.Z_path,'+r','linewidth',3) 
% end

if(A.path_counter <= length(A.X_path))
   A.X_des_EF = A.X_path(A.path_counter);
   A.Y_des_EF = A.Y_path(A.path_counter);
   A.Z_des = A.Z_path(A.path_counter);
else
    plot_XY;
    A.flagggg=1;
end
% A.X_des_EF
% A.Y_des_EF
% A.path_counter
% A.X_kalman
% abs(A.X_kalman-A.X_des)<.02
if((abs(A.X_kalman-A.X_des_EF)<.04)&&(abs(A.Y_kalman-A.Y_des_EF)<.04)&&(abs(A.Z_kalman-A.Z_des)<.08))
    A.path_counter = A.path_counter+1;
end