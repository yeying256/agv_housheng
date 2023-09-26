figure(1)
t=1:10000;
subplot(3,1,1);
plot(t,xyw_cmd_(1,:),t,xyw_cmd_(2,:),t,xyw_cmd_(3,:));
legend('vx','vy','wz');

subplot(3,1,2);
plot(t,cmd_turn_vel(1,:),t,cmd_turn_vel(2,:));
legend('cmd_turn_vel1','cmd_turn_vel2');

subplot(3,1,3);
plot(t,cmd_wheel_vel(1,:),t,cmd_wheel_vel(2,:));
legend('cmd_wheel_vel1','cmd_wheel_vel2');
