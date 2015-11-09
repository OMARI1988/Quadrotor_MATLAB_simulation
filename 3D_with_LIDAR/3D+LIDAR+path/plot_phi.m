% this function will plot the Z value with the desired one. 
global A
figure
axes('fontsize',14)

hold on
% plot(A.t_plot(1:A.counter),A.phi_plot(1:A.counter)+A.phi_error(1:A.counter),'y')
plot(A.t_plot(1:A.counter),A.phi_plot(1:A.counter),'r')
plot(A.t_plot(1:A.counter),A.phi_ref_plot(1:A.counter),'b')
% plot(A.t_plot(1:A.counter),A.phi_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.phi_kalman_plot(1:A.counter),'color',[1 .4 .8])
% legend('measured response','actual response','set value','disturbances','kalman filter')

% cla
hold on
% plot(A.t_plot(1:A.counter),A.X_plot(1:A.counter)+A.X_error(1:A.counter),'y')
% plot(A.t_plot(1:A.counter-1),A.X_plot(1:A.counter-1),'r','linewidth',1)
% plot(A.t_plot(1:A.counter-1),A.X_ref_plot(1:A.counter-1),'b')
% plot(A.t_plot(1:A.counter),A.X_dis_plot(1:A.counter),'g')
% plot(A.t_plot(1:A.counter),A.X_kalman_plot(1:A.counter),'color',[1 .4 .8])
a=legend('actual response','set value')
set(a,'fontsize',14)

xlabel('Time (s)','fontsize',14)
ylabel('phi (rad)','fontsize',14)
title('phi vs. time','fontsize',14)