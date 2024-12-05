% figure(1);
% clf;
% subplot(3,1,1);
% hold on
% title('q1 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos1, 'r.', 'DisplayName', 'q1');
% 
% subplot(3,1,2);
% hold on
% title('q2 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos2, 'r.', 'DisplayName', 'q2');
% 
% subplot(3,1,3);
% hold on
% title('q2 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos3, 'r.', 'DisplayName', 'q3');



figure(1);
hold on;
plot(t, pos1, 'r.', 'DisplayName', 'q1');
plot(t, pos2, 'g.', 'DisplayName', 'q2');
plot(t, pos3, 'b.', 'DisplayName', 'q3');
name = strcat("Brake release: ", brake_name, "s, Retract knee: ", retract_name, "s, Stopping angle: ", string(pos3(end)), " rad");
xlabel('Time (s)');
ylabel('Angle (rad)');
title(name)
legend;


% load("run9.mat")
% 
% figure(1);
% clf;
% subplot(3,1,1);
% hold on
% title('q1 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos1, 'r.', 'DisplayName', 'q1');
% plot(t, pos1_des, 'b-', 'DisplayName', 'q1_{des}');
% scatter(0.4675, 0.03, 'LineWidth', 5)
% 
% subplot(3,1,2);
% hold on
% title('q2 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos2, 'r.', 'DisplayName', 'q2');
% plot(t, pos2_des, 'b-', 'DisplayName', 'q2_{des}');
% scatter(0.4675, -0.12, 'LineWidth', 5)
% 
% subplot(3,1,3);
% hold on
% title('q3 vs t');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('show');
% grid on;
% plot(t, pos3, 'r.', 'DisplayName', 'q3');
% plot(t, pos3_des, 'b-', 'DisplayName', 'q3_{des}');
% scatter(0.4675, 0.4, 'LineWidth', 5)

% scatter3(SweepData.kneeDrawTime(19:32,1), SweepData.brakeReleaseTime(19:32,1), SweepData.distance(19:32,1))
 % plot3(SweepData.kneeDrawTime(9:18,1), SweepData.brakeReleaseTime(9:18,1), SweepData.distance(9:18,1))


% x = SweepData.kneeDrawTime(19:32,1);
% y = SweepData.brakeReleaseTime(19:32,1);
% z = SweepData.distance(19:32,1);

% x = SweepData.kneeDrawTime(9:18,1);
% y = SweepData.brakeReleaseTime(9:18,1);
% z = SweepData.distance(9:18,1);

% x = SweepData.kneeDrawTime(1:9,1);
% y = SweepData.brakeReleaseTime(1:9,1);
% z = SweepData.distance(1:9,1);
% 
% F = TriScatteredInterp(x,y,z);
% % ti = 0.5:.01:8;
% ti = 0.3:.01:7;
% [qx,qy] = meshgrid(ti,ti);
% qz = F(qx,qy);
% mesh(qx,qy,qz)
% hold on
% scatter3(x,y,z,'o')
% xlabel("Knee Draw Time (s)");
% ylabel('Brake Release Time (s)');
% zlabel('Stopping angle (rad)')