% Chosen link dimensions
r1 = 130;
r2 = 120;
r3 = 100;
r4 = 15;
r5 = 105.63;    %measured in SolidWorks

x0 = [0 0]; % for solving system of loop closure equations

theta_4=0:0.0873:6.3705; % theta 4 can make a full 360 deg 
theta11 = deg2rad(30); 
theta12 = deg2rad(60);
% ================== MOTION OF JOINT B (END EFFECTOR) ====================
% Plot 1a: End effector coordinates for theta_1 = 30 degrees
for i=1:1:73
    current_theta_4 = theta_4(i);
    xsol = fsolve(@(x)equations(x,theta11, current_theta_4, r1, r2, r3, r4, r5),x0);
    theta_2(i) = xsol(1); %#ok<*NOPTS> % rad
    theta_3(i) = xsol(2); % rad
    
    xC1(i) = r4 * cos(current_theta_4) + r3 * cos(theta_3(i));
    yC1(i) = r4 * sin(current_theta_4) + r3 * sin(theta_3(i));    
end

figure;
plot(xC1,yC1,'k-')
title('End effector coordinates for theta_1 = 30 degrees');
h=legend('Path of end effectors');
xlabel('x - coordinate (mm)'),ylabel('y - coordinate (mm)');

% Plot 1b: End effector coordinates for theta_1 = 60 degrees
for i=1:1:73
    current_theta_4 = theta_4(i);
    xsol = fsolve(@(x)equations(x,theta12, current_theta_4, r1, r2, r3, r4, r5),x0);
    theta_2(i) = xsol(1); %#ok<*NOPTS> % rad
    theta_3(i) = xsol(2); % rad
    
    xC2(i) = r4 * cos(current_theta_4) + r3 * cos(theta_3(i));
    yC2(i) = r4 * sin(current_theta_4) + r3 * sin(theta_3(i));    
end
figure;
plot(xC2,yC2,'k-')
title('End effector coordinates for theta_1 = 60 degrees');
h=legend('Path of end effectors');
xlabel('x - coordinate (mm)'),ylabel('y - coordinate (mm)');


% ======================= MOTION OF JOINT A ==============================
% Plot 1a: End effector coordinates for theta_1 = 30 degrees
theta1_values = 0:0.08725:1.047; %theta1 in range [0, 60] degrees
for i=1:1:12
    xC3(i) = r1 * cos(theta1_values(i));
    yC3(i) = r1 * sin(theta1_values(i));
end
figure;
plot(xC3,yC3,'k-')
title('Joint A coordinates for theta_1 in the range [0, 60] degrees');
h=legend('Path of Joint A');
xlabel('x - coordinate (mm)'),ylabel('y - coordinate (mm)');

% Function that solves system of equations for theta_2 and theta_3
function F=equations(x,theta_1, theta_4, r1, r2, r3, r4, r5)
% x(1): theta_2
% x(2): theta_3

F(1) = r1*cos(theta_1) + r2*cos(x(1)) - r3*cos(x(2)) - r4*cos(theta_4) ; 
F(2) = r1*sin(theta_1) + r2*sin(x(1)) - r3*cos(x(2)) * r4*sin(theta_4) - r5;
 
end