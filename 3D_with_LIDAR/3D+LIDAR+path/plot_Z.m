% this function will plot the Z value with the desired one. 
global A
figure
% % cla
% hold on
% plot(A.t_plot(1:A.counter),A.Z_plot(1:A.counter)+A.Z_error(1:A.counter),'y')
% plot(A.t_plot(1:A.counter),A.Z_plot(1:A.counter),'r','linewidth',1)
% plot(A.t_plot(1:A.counter),A.Z_ref_plot(1:A.counter),'b')
% plot(A.t_plot(1:A.counter),A.Z_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.Z_kalman_plot(1:A.counter),'color',[1 .4 .8])
% legend('measured response','actual response','set value','disturbances','kalman filter')
% 
% xlabel('time (s)')
% ylabel('altitude (m)')
% title('altitude vs. time')

axes('fontsize',14)
% cla
hold on
% plot(A.t_plot(1:A.counter),A.X_plot(1:A.counter)+A.X_error(1:A.counter),'y')
plot(A.t_plot(1:A.counter-1),A.Z_plot(1:A.counter-1),'r','linewidth',1)
plot(A.t_plot(1:A.counter-1),A.Z_ref_plot(1:A.counter-1),'b')
% plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
a=legend('actual response','set value')
set(a,'fontsize',14)

xlabel('Time (s)','fontsize',14)
ylabel('Z axis (meter)','fontsize',14)
title('Z axis vs. time','fontsize',14)