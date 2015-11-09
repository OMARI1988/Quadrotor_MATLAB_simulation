% this function will plot the Z value with the desired one. 
global A
figure
% cla
hold on

plot3(A.X_plot(1:A.counter-1),A.Y_plot(1:A.counter-1),A.Z_plot(1:A.counter-1))
plot3(A.X_path,A.Y_path,A.Z_path,'+r','linewidth',3) 
view(70,50)
grid on
axis equal
axis([-4 2 -1 3 0 2])
% plot(A.t_plot(1:A.counter),A.Y_plot(1:A.counter),'r','linewidth',1)
% plot(A.t_plot(1:A.counter),A.Y_ref_plot(1:A.counter),'b')
% plot(A.t_plot(1:A.counter),A.Y_dis_plot(1:A.counter),'g')

for i=1:A.num_obstacles(1)
patch([A.B1(i,1) A.C1(i,1) A.C1(i,1) A.B1(i,1)],[A.B1(i,2) A.C1(i,2) A.C1(i,2) A.B1(i,2)],[A.B1(i,3) A.B1(i,3) A.D1(i,3) A.D1(i,3)],[0 .4 1])
end

legend('Quadrotor path','Desired path')

xlabel('X axis (m)')
ylabel('Y axis (m)')
zlabel('Z axis (m)')

title('path traveled')