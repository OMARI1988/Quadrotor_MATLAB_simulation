% this function will plot the Z value with the desired one. 
global A
figure
axes('fontsize',14)

hold on
plot(A.t_plot(1:A.counter),A.theta_plot(1:A.counter)+A.theta_error(1:A.counter),'color',[.8 .8 .8])
plot(A.t_plot(1:A.counter),A.theta_plot(1:A.counter),'r')
plot(A.t_plot(1:A.counter),A.theta_ref_plot(1:A.counter),'b')
plot(A.t_plot(1:A.counter),A.theta_dis_plot(1:A.counter),'g')
plot(A.t_plot(1:A.counter),A.theta_kalman_plot(1:A.counter),'color',[0 1 0])
a=legend('actual response','set value')
set(a,'fontsize',14)

xlabel('Time (s)','fontsize',14)
ylabel('theta (rad)','fontsize',14)
title('theta vs. time','fontsize',14)


