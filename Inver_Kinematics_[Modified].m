clc
clear
syms th1 th2 nx ny nz ox oy oz ax ay az Px Py Pz a1 a2 th1_deg th2_deg

RotZ1 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 0; 0 0 0 1];
RotZ2 = [cos(th2) -sin(th2) 0 0; sin(th2) cos(th2) 0 0; 0 0 1 0; 0 0 0 1];

TransA1 = [1 0 0 a1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
TransA2 = [1 0 0 a2; 0 1 0 0; 0 0 1 0; 0 0 0 1];

T_0_2 = RotZ1*TransA1*RotZ2*TransA2;
TT = [nx ox ax Px; ny oy ay Py; nz oz az Pz; 0 0 0 1];

% Left Step 1
LS1 = simplify(inv(RotZ1)*TT);

% Right Step 1
RS1 = simplify(inv(RotZ1)*T_0_2);

%%=================%%
% set alpha for motor_1
alpha_deg=20;
% set beta for motor_2
beta_deg=20;
%%=================%%

%% link(common) 
a1 = 7.5;
a2 = 11;
L=10.5;

% offset ignore (common)
offset=2;
%H=a2+offset;
H=a2;


%set max motor_1 deg
max_motor_1_rad=abs(90)*((2*pi)/360);
index=1;

%end point for motor 1
Px = a1-L*(1-cos(alpha_deg*((2*pi)/360)));
Py = H+L*sin(alpha_deg*((2*pi)/360));

%sol for motor 1
th2 = [2*atan((((a1+a2)^2-(Px^2+Py^2))/(Px^2+Py^2-(a1-a2)^2))^0.5) -2*atan((((a1+a2)^2-(Px^2+Py^2))/(Px^2+Py^2-(a1-a2)^2))^0.5)];

th1 = atan2(Py,Px) - atan(a2*sin(th2)./(a1+a2*cos(th2)));

%determine sol for motor 1
for i=1:2
if(abs(th1(i))<=max_motor_1_rad)
    index=i;
end

end

th2_deg=th2(index)*(180/pi);
th1_deg=th1(index)*(180/pi);

%%=======================================motor2

%set max motor_2 deg
max_motor_2_rad=abs(90)*((2*pi)/360);
index_2=1;

%end point for motor 2
Px_2 = a1-L*(1-cos(beta_deg*((2*pi)/360)));
Py_2 = H+L*sin(beta_deg*((2*pi)/360));

%sol for motor 2
th2_2 = [2*atan((((a1+a2)^2-(Px_2^2+Py_2^2))/(Px_2^2+Py_2^2-(a1-a2)^2))^0.5) -2*atan((((a1+a2)^2-(Px_2^2+Py_2^2))/(Px_2^2+Py_2^2-(a1-a2)^2))^0.5)];
th1_2 = atan2(Py_2,Px_2) - atan(a2*sin(th2_2)./(a1+a2*cos(th2_2)));

%determine sol for motor 2
for i=1:2
if(abs(th1_2(i))<=max_motor_2_rad)
    index_2=i;
end

end

th2_deg_2=th2_2(index_2)*(180/pi);
th1_deg_2=th1_2(index_2)*(180/pi);



th1_deg, 

th1_deg_2, 





%==print==
RotZ = @(theta) [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0 ; 0 0 0 1];
Trans = @(x, y, z) [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];


    theta1 = th1(index);
    theta2 = th2(index);
    
    T_0_1 = RotZ(theta1)*Trans(a1, 0, 0);
    T_1_2 = RotZ(theta2)*Trans(a2, 0, 0);
    T_total = T_0_1*T_1_2;
    
    y0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    y0_1 = T_0_1 * y0;
    y0_2 = T_total * y0;
    
    originX = [y0(1,4), y0_1(1,4), y0_2(1,4)];
    originY = [y0(2,4), y0_1(2,4), y0_2(2,4)];
    
    figure
    plot(originX, originY, '*--', 'linewidth', 2);
    axis([min([originX, originY])-4, max([originX, originY])+4, min([originX, originY])-4, max([originX, originY])+4])
    grid on
    axis square
    
    axisY0X = y0(1, 1:2);
    axisY0_1X = y0_1(1, 1:2);
    axisY0_2X = y0_2(1, 1:2);
    
    axisY0Y = y0(2, 1:2);
    axisY0_1Y = y0_1(2, 1:2);
    axisY0_2Y = y0_2(2, 1:2);
    title('For motor 1')

    %%=============
    

    theta1 = th1_2(index_2);
    theta2 = th2_2(index_2);
    
    T_0_1 = RotZ(theta1)*Trans(a1, 0, 0);
    T_1_2 = RotZ(theta2)*Trans(a2, 0, 0);
    T_total = T_0_1*T_1_2;
    
    y0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    y0_1 = T_0_1 * y0;
    y0_2 = T_total * y0;
    
    originX = [y0(1,4), y0_1(1,4), y0_2(1,4)];
    originY = [y0(2,4), y0_1(2,4), y0_2(2,4)];
    
    figure
    plot(originX, originY, '*--', 'linewidth', 2);
    axis([min([originX, originY])-4, max([originX, originY])+4, min([originX, originY])-4, max([originX, originY])+4])
    grid on
    axis square
    
    axisY0X = y0(1, 1:2);
    axisY0_1X = y0_1(1, 1:2);
    axisY0_2X = y0_2(1, 1:2);
    
    axisY0Y = y0(2, 1:2);
    axisY0_1Y = y0_1(2, 1:2);
    axisY0_2Y = y0_2(2, 1:2);
    title('For motor 2')