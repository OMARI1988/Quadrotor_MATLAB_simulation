% this function will plot the Z value with the desired one. 
global A
cla
hold on
subplot(2,2,1)
plot(A.t_plot(1:A.counter),A.O1_plot(1:A.counter),'r')
a=legend('front')
set(a,'fontsize',14)
xlabel('time (s)','fontsize',14)
ylabel('motor speed (rpm)','fontsize',14)

subplot(2,2,2)
plot(A.t_plot(1:A.counter),A.O2_plot(1:A.counter),'g')
a=legend('right')
set(a,'fontsize',14)
xlabel('time (s)','fontsize',14)
ylabel('motor speed (rpm)','fontsize',14)

subplot(2,2,3)
plot(A.t_plot(1:A.counter),A.O3_plot(1:A.counter),'b')
a=legend('rear')
set(a,'fontsize',14)
xlabel('time (s)','fontsize',14)
ylabel('motor speed (rpm)','fontsize',14)

subplot(2,2,4)
plot(A.t_plot(1:A.counter),A.O4_plot(1:A.counter),'k')
a=legend('left')
set(a,'fontsize',14)
xlabel('time (s)','fontsize',14)
ylabel('motor speed (rpm)','fontsize',14)

% title('Forces and Torques vs. time')