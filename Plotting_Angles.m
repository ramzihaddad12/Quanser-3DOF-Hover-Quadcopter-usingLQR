figure(1)
yaw = getsampleusingtime(Yaw_Angle,10,20);
plot(yaw)
title('Yaw Angle for a Step Input')
xlabel('Time (sec)')
ylabel('Angle (deg)')

figure(2)
pitch = getsampleusingtime(Pitch_Angle,10,20);
plot(pitch)
% axis([10 20],[-5 5])
title('Pitch Angle for a Step Input')
xlabel('Time (sec)')
ylabel('Angle (deg)')