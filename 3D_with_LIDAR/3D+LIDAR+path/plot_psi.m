% this function will plot the Z value with the desired one. 
global A
figure
% hold on
% plot(A.t_plot(1:A.counter),A.psi_plot(1:A.counter)+A.psi_error(1:A.counter),'y')
% plot(A.t_plot(1:A.counter),A.psi_plot(1:A.counter),'r')
% plot(A.t_plot(1:A.counter),A.psi_ref_plot(1:A.counter),'b')
% plot(A.t_plot(1:A.counter),A.psi_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.psi_kalman_plot(1:A.counter),'color',[1 .4 .8])
% legend('measured response','actual response','set value','disturbances','kalman filter')
% xlabel('time (s)')
% ylabel('psi (rad)')
% title('psi vs. time')
axes('fontsize',14)

hold on
plot(A.t_plot(1:A.counter),A.psi_plot(1:A.counter),'r')
plot(A.t_plot(1:A.counter),A.psi_ref_plot(1:A.counter),'b')

a=legend('actual response','set value')
set(a,'fontsize',14)

xlabel('Time (s)','fontsize',14)
ylabel('psi (rad)','fontsize',14)
title('psi vs. time','fontsize',14)