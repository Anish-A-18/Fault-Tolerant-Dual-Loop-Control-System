
clear;

% g = 9.81;       % gravity m/s2
% m = 0.468;      % mass kg
% l = 0.225;      % distance between center of mass and rotors
% k = 2.980e-6;   % lift constant
% b = 1.140e-7;   % drag constant
% IM = 3.357e-5;  % Moment of Inertia of rotor kg*m2
% Ixx = 4.856e-3; % diagnal of Inertia matrix for x kg*m2
% Iyy = 4.856e-3; % diagnal of Inertia matrix for y kg*m2
% Izz = 8.801e-3; % diagnal of Inertia matrix for z kg*m2
% Ax = 0.25;      % drag force coefficients for velocity kg/s
% Ay = 0.25;      % drag force coefficients for velocity kg/s
% Az = 0.25;      % drag force coefficients for velocity kg/s
% Ir =  1/12*m*l*l;

g = 9.81;       % gravity m/s2
m = 0.600;      % mass kg
l = 0.254;      % distance between center of mass and rotors
k = 2.980e-6;   % lift constant
b = 1.140e-7;   % drag constant
IM = 3.357e-5;  % Moment of Inertia of rotor kg*m2
Ixx = 9.25e-3; % diagnal of Inertia matrix for x kg*m2
Iyy = 9.25e-3; % diagnal of Inertia matrix for y kg*m2
Izz = 8.801e-3; % diagnal of Inertia matrix for z kg*m2
Ax = 0.25;      % drag force coefficients for velocity kg/s
Ay = 0.25;      % drag force coefficients for velocity kg/s
Az = 0.25;      % drag force coefficients for velocity kg/s
Ir = 3.357e-5;   % inertia of the motors

%% Time definition (20ms for 80sec)
timeend = 80;
timepoints = 4000+1;
tspan = linspace(0,timeend,timepoints);
tdel = timeend/(timepoints-1);

f_dist = 1;
x_amp_dist = 0.01;
phi_amp_dist = 0.01;
pi = 3.14;

% x_acc = zeros(1,timepoints);
% y_acc = zeros(1,timepoints);
% z_acc = zeros(1,timepoints);
%
% q_acc = zeros(1,timepoints);
% p_acc = zeros(1,timepoints);
% r_acc = zeros(1,timepoints);
%
% theta_acc = zeros(1,timepoints);
% phi_acc = zeros(1,timepoints);
% psi_acc = zeros(1,timepoints);

xv = zeros(1,timepoints);
yv = zeros(1,timepoints);
zv = zeros(1,timepoints);

q = zeros(1,timepoints);
p = zeros(1,timepoints);
r = zeros(1,timepoints);

theta_v = zeros(1,timepoints);
phi_v = zeros(1,timepoints);
psi_v = zeros(1,timepoints);

x = zeros(1,timepoints);
y = zeros(1,timepoints);
z = zeros(1,timepoints);

theta = zeros(1,timepoints);
phi = zeros(1,timepoints);
psi = zeros(1,timepoints);

%% PD/PID controller constants
% KxD = 8; %6; %4;
% KxP = 6;
% KxI = .5; %0; %.5;
% KyD = 8;
% KyP = 6;
% KyI = .5; %0; %.5;
% KzD = 2;
% KzP = 6;
% KzI = .0; %0; %.5;
% KphiD = 2;
% KphiP = 6;
% KphiI = .0; %0; %.5;
% KthetaD = 2;
% KthetaP = 6;
% KthetaI = .0; %0; % .5;
% KpsiD = 2;
% KpsiP = 6;
% KpsiI = 1; %0; %.5

%% PD
% KxD = 10; %6; %4;
% KxP = 6;
% KyD = 10;
% KyP = 6;
% KzD = 10;
% KzP = 6;
% KphiD = 6;
% KphiP = 8;
% KthetaD = 6;
% KthetaP = 8;
% KpsiD = 2;
% KpsiP = 6;

%% FSOM controller constants
% Kz = 10;
% Kx = 30;
% Ky = 30;
% Kphi = 5;
% Ktheta = 5;
% Kpsi = 5;
% Kz = 20;
% Kx = 50;
% Ky = 50;
% Kphi = 10;
% Ktheta = 10;
% Kphi_xy = 15;
% Ktheta_xy = 15;
% Kpsi = 10;

%% SMC controller constants
% Kz = 1.5; %2
% Kz1 = 2;
% Kz2 = 2;
% Kx = 1;
% Kx1 = 2;
% Kx2 = 2; %1
% Ky = 1;
% Ky1 = 2;
% Ky2 = 2; %1
%  Kphi = 2;
% Kphi1 = 4;
% Kphi2 =2; %.5
%  Ktheta = 2;
% Ktheta1 = 4;
% Ktheta2 = 2; %.5
% Kphi1_xy = 20; %15
% Kphi2_xy = 8;
% Ktheta1_xy = 20; %15
% Ktheta2_xy = 8;                
% Kpsi = 2;
% Kpsi1 = 2;
% Kpsi2 = 1;

%% BSC controller constants
% Kz1 = 2;
% Kz2 = 2;
% Kx1 = 2;
% Kx2 = 1;
% Ky1 = 2;
% Ky2 = 1;
% Kphi1 = 8;%15;
% Kphi2 = 4;%8;
% Ktheta1 = 8;%15;
% Ktheta2 = 4;%8;
% Kphi1_xy = 16;
% Kphi2_xy = 8;
% Ktheta1_xy = 16;
% Ktheta2_xy = 8;
% Kpsi1 = 2;
% Kpsi2 = 2;

%% initial conditions
z(1) = .5;
x(1) = .5;
y(1) = .5;
theta(1) = 10*pi/180;
phi(1) =  20*pi/180;
psi(1) = 10*pi/180;

%% desired values

thetad = 0;
phid = 0;
psid = 0;
zvd = 0;
theta_vd = 0;
phi_vd = 0;
psi_vd = 0;
xvd =0;
yvd=0;

%xd = y2;
%yd = y2;
xd(1:2000) = 0;
xd(2001:2500) = 15;
%xd(4001:6000) = 10;
xd(2501:4001) = 15;
yd(1:2000) = 0;
yd(2001:2500) = 15;
%yd(4001:6000) = 10;
yd(2501:4001) = 15;
zd = 0;
zdx(1:4001) = 0;
%
%   xd = 0.2*tspan.*tspan;
%
% % % % %xd(70001:80000) = xd(70000);
%   yd = 0.1*tspan.*tspan;
% % % %yd(60001:80001) = yd(70000);

% T = zeros(1,timepoints);
%
% T_PD = zeros(1,timepoints);
% torq_phi_PD = zeros(1,timepoints);
% torq_theta_PD = zeros(1,timepoints);
% torq_psi_PD = zeros(1,timepoints);
w1 = zeros(1,timepoints);
w2 = zeros(1,timepoints);
w3 = zeros(1,timepoints);
w4 = zeros(1,timepoints);

eI_x = zeros(1,timepoints);
eI_y = zeros(1,timepoints);
eI_z = zeros(1,timepoints);
eI_phi = zeros(1,timepoints);
eI_theta = zeros(1,timepoints);
eI_psi = zeros(1,timepoints);
% w1_2 = zeros(1,timepoints);
% w2_2 = zeros(1,timepoints);
% w3_2 = zeros(1,timepoints);
% w4_2 = zeros(1,timepoints);
max_w = 2000;

xm = zeros(5,timepoints);
ym = zeros(5,timepoints);
zm = zeros(5,timepoints);


% phi_deg = 10;    % roll angle
% theta_deg = 10; % pitch angle
% psi_deg = 10;    % yaw angle
%
% psi_rad = psi_deg*pi/180;
% phi_rad = phi_deg*pi/180;
% theta_rad = theta_deg*pi/180;
%
% w1 = 20;
% w2 = 20;
% w3 = 20;
% w4 = 20;
%
% % R11 = cos(psi_rad)*cos(phi_rad)-cos(theta_rad)*sin(psi_rad)*sin(phi_rad);
% % R12 = -sin(psi_rad)*cos(phi_rad)-cos(theta_rad)*sin(psi_rad)*cos(phi_rad);
% % R13 = sin(theta_rad)*sin(psi_rad);
% % R21 = sin(psi_rad)*cos(phi_rad)+cos(theta_rad)*cos(psi_rad)*sin(phi_rad);
% % R22 = cos(theta_rad)*cos(psi_rad)*cos(phi_rad)-sin(psi_rad)*sin(phi_rad);
% % R23 = -cos(psi_rad)*sin(theta_rad);
% % R31 = sin(theta_rad)*sin(phi_rad);
% % R32 = sin(theta_rad)*cos(phi_rad);
% % R33 = cos(theta_rad);
%
%
% % Right hand rotation-- rotation from body to local frame
% % Local frame is Same orientation as world frame with the origin at the
% % Center of Mass of Quad
% % Body frame is with respect to Quadcopter
% % RotM_Inv is local to body which is transpose of RotM
%
% R11 = cos(psi_rad)*cos(theta_rad);
% R12 = -sin(psi_rad)*cos(phi_rad)+sin(theta_rad)*sin(phi_rad)*cos(psi_rad);
% R13 = sin(psi_rad)*sin(phi_rad)+sin(theta_rad)*cos(phi_rad)*cos(psi_rad);
% R21 = cos(theta_rad)*cos(psi_rad);
% R22 = sin(theta_rad)*sin(psi_rad)*sin(phi_rad)+cos(psi_rad)*cos(phi_rad);
% R23 = sin(theta_rad)*sin(psi_rad)*cos(phi_rad)-cos(psi_rad)*sin(phi_rad);
% R31 = -sin(theta_rad);
% R32 = cos(theta_rad)*sin(phi_rad);
% R33 = cos(theta_rad)*cos(phi_rad);
%
% RotM = [ R11 R12 R13; R21 R22 R23; R31 R32 R33];
%
% RotM_Inv = [ R11 R21 R31; R12 R22 R32; R13 R23 R33];
%
%
% % Transformation matrix for angular velocities from inertial local) frame to
% % body frame
%
% W_inv11 = 1;
% W_inv12 = sin(phi_rad)*tan(theta_rad);
% W_inv13 = cos(phi_rad)*tan(theta_rad);
% W_inv21 = 0;
% W_inv22 = cos(phi_rad);
% W_inv23 = -sin(phi_rad);
% W_inv31 = 0;
% W_inv32 = sin(phi_rad)/cos(theta_rad);
% W_inv33 = cos(phi_rad)/cos(theta_rad);
%
% W_inv = [ W_inv11 W_inv12 W_inv13; W_inv21 W_inv22 W_inv23; W_inv31 W_inv32 W_inv33];
%
%
% W_11 = 1;
% W_12 = 0;
% W_13 = sin(phi_rad)/cos(theta_rad);
% W_21 = 0;
% W_22 = cos(phi_rad);
% W_23 = sin(phi_rad)*cos(theta_rad);
% W_31 = 0;
% W_32 = -sin(phi_rad);
% W_33 = cos(phi_rad)*cos(theta_rad);
%
%
% W = [ W11 W12 W13; W21 W22 W23; W31 W32 W33];
%
% % Quadcopter Inertial matrix
% IntM =[Ixx 0 0; 0 Iyy 0; 0 0 Izz];
% IntM_inv = [1/Ixx 0 0; 0 1/Iyy 0; 0 0 1/Izz ];
%
% f1 = k*w1*w1;
% f2 = k*w2*w2;
% f3 = k*w3*w3;
% f4 = k*w4*w4;
%
% torqM1 = b*w1*w1 + IM*dev_w1*dev_w1;
% torqM2 = b*w2*w2 + IM*dev_w2*dev_w2;
% torqM3 = b*w4*w3 + IM*dev_w3*dev_w3;
% torqM4 = b*w4*w4 + IM*dev_w4*dev_w4;
% torqM1_simp = b*w1*w1;
% torqM2_simp = b*w2*w2;
% torqM3_simp = b*w4*w3;
% torqM4_simp = b*w4*w4;
%
%
% Thrust_base = [0; 0; f1+f2+f3+f4];
% torq_base = [ l*(-f2+f4); l*(-f1+f3); torqM1+torqM2+torqM3+torqM4];


%
% w1(1:(timepoints-1)/4) = 625+50*sin(2*pi*2*tspan(1:10000));
% w1((timepoints-1)/4+1:2*(timepoints-1)/4) = 625+0*sin(2*pi*2*tspan(10001:20000));
% w1(2*(timepoints-1)/4+1:3*(timepoints-1)/4) = 625-25*sin(2*pi*2*tspan(20001:30000));
% w1(3*(timepoints-1)/4+1:4*(timepoints-1)/4+1) = 625-25*sin(2*pi*2*tspan(30001:40001));
%
% w2(1:10000) = 625+50*sin(2*pi*2*tspan(1:10000));
% w2(10001:20000) = 625-25*sin(2*pi*2*tspan(10001:20000));
% w2(20001:30000) = 625+0*sin(2*pi*2*tspan(20001:30000));
% w2(30001:40001) = 625+25*sin(2*pi*2*tspan(30001:40001));
%
% w3(1:10000) = 625+50*sin(2*pi*2*tspan(1:10000));
% w3(10001:20000) = 625+0*sin(2*pi*2*tspan(10001:20000));
% w3(20001:30000) = 625+25*sin(2*pi*2*tspan(20001:30000));
% w3(30001:40001) = 625-25*sin(2*pi*2*tspan(30001:40001));
%
% w4(1:10000) = 625+50*sin(2*pi*2*tspan(1:10000));
% w4(10001:20000) = 625+25*sin(2*pi*2*tspan(10001:20000));
% w4(20001:30000) = 625+0*sin(2*pi*2*tspan(20001:30000));
% w4(30001:40001) = 625+25*sin(2*pi*2*tspan(30001:40001));
%
% T = k*(w1.*w1+w2.*w2+w3.*w3+w4.*w4);

% linear dynamics
startloop = 1;

for pp=1:1
for qq=1:1  
for rr=1:1
for ss=1:1
% 
% Kz1 = 4;
% Kz2 = 4;
% Kx1 = .5;
% Kx2 = 3;
% Ky1 = .5;
% Ky2 = 3;
% Kphi1 = 8;%15;
% Kphi2 = 4;%8;
% Ktheta1 = 8;%15;
% Ktheta2 = 4;%8;
% Kphi1_xy = 16;
% Kphi2_xy = 10;
% Ktheta1_xy = 16;
% Ktheta2_xy = 10;
% Kpsi1 = 4;
% Kpsi2 = 4;
% % 
% KxD =4; %6; %4;
% KxP = 4;
% KxI = 1; %0; %.5;
% KyD = 4;
% KyP = 4;
% KyI = 1; %0; %.5;
% KzD = 4;
% KzP = 5;
% KzI = 1; %0; %.5;
% KphiD = 4;
% KphiP = 8;
% KphiI = 4; %0; %.5;
% KthetaD = 4;
% KthetaP = 8;
% KthetaI = 4; % %0; % .5;
% KpsiD = 6;
% KpsiP = 8;
% KpsiI = 4; %0; %.5;
% KphiDx = 4;
% KphiPx = 8;
% KphiIx = 4; %0; %.5;
% KthetaDx = 4;
% KthetaPx = 8;
% KthetaIx = 4; % %0; % .5;

% Kz = 20;
% Kx = 50;
% Ky = 50;
% Kphi = 10;
% Ktheta = 10;
% Kpsi = 10;

Kz = 2; %2
Kz1 = 1;
Kz2 = 5;
Kx = .5;
Kx1 = 4;
Kx2 = 4; %1
Ky = .5;
Ky1 = 4;
Ky2 = 4; %1
 Kphi = 2;
Kphi1 = 6;
Kphi2 =6;%2; %.5
 Ktheta = 2;
Ktheta1 = 6;
Ktheta2 = 6; %2; %.5
Kphi1_xy = 30; %15
Kphi2_xy = 30;
Ktheta1_xy = 30; %15
Ktheta2_xy = 30;                
Kpsi = 1;
Kpsi1 = 1;
Kpsi2 = 3;
% Kz = 2; %2
% Kz1 = 1;
% Kz2 = 5;
% Kx = 1;
% Kx1 = 3;
% Kx2 = 3; %1
% Ky = 1;
% Ky1 = 3;
% Ky2 = 3; %1
%  Kphi = 2;
% Kphi1 = 4;
% Kphi2 =2;%2; %.5
%  Ktheta = 2;
% Ktheta1 = 4;
% Ktheta2 = 2; %2; %.5
% Kphi1_xy = 10; %15
% Kphi2_xy = 10;
% Ktheta1_xy = 10; %15
% Ktheta2_xy = 10;                
% Kpsi = 1;
% Kpsi1 = 1;
% Kpsi2 = 3;

    for addnoise = 1:2
        n = 0;
        for t = 2:timepoints

            n = n+1 ;
            [x(t),y(t),z(t),xv(t),yv(t),zv(t),phi(t),theta(t),psi(t),...
                phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t)] = ...
                quad_in_imbalance(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1(t-1),w2(t-1),w3(t-1),w4(t-1),...
                x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
                zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));


            if addnoise == 2
                %% Disturbance in time
                x(t) = x(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
                y(t) = y(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
                z(t) = z(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
                phi(t) = phi(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);
                theta(t) = theta(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);
                psi(t) = psi(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);

                %% Addition of Noise
                x(t) = x(t)+x_amp_dist*randn;
                y(t) = y(t)+x_amp_dist*randn;
                z(t) = z(t)+x_amp_dist*randn;
                phi(t) = phi(t)+phi_amp_dist*randn;
                theta(t) = theta(t)+phi_amp_dist*randn;
                psi(t) = psi(t)+phi_amp_dist*randn;
            end

            %% PD
%             [w1(t),w2(t),w3(t),w4(t)] = pd1(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,...
%                 KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv(t),z(t),...
%                 phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
%                 psi_vd,psid,psi_v(t),psi(t));

            %% PID
%             [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
%                 pid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),eI_y(t-1),...
%                 KzD,KzP,KzI,eI_z(t-1),KphiD,KphiP,KphiI, ...
%                 eI_phi(t-1),KthetaD,KthetaP,KthetaI,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
%                 max_w,xd(t),x(t),yd(t),y(t),zvd,zd,zv(t),z(t),...
%                 phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


            %% FSOM
%             [w1(t),w2(t),w3(t),w4(t)] = fsom1(m,l,k,b,Ixx,Iyy,Izz,Kz,Kphi,...
%                 Ktheta,Kpsi,max_w,zvd,zd,zv(t),z(t),...
%                 phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
%                 psi_vd,psid,psi_v(t),psi(t));

            %% BSC
%             [w1(t),w2(t),w3(t),w4(t)] = bsc1(m,l,k,b,Ixx,Iyy,Izz,Kz1,Kz2, Kphi1, ...
%                 Kphi2,Ktheta1,Ktheta2,Kpsi1,Kpsi2,max_w,zvd,zd,zv(t),z(t),z(t-1),...
%                 phi_vd,phid,phi_v(t),phi(t),phi(t-1),theta_vd,thetad,theta_v(t),theta(t),theta(t-1),...
%                 psi_vd,psid,psi_v(t),psi(t),psi(t-1));

            % SMC
            [w1(t),w2(t),w3(t),w4(t)] = smc1(m,l,k,b,Ixx,Iyy,Izz,Kz,Kz1,Kz2, Kphi,Kphi1, ...
                Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,zvd,zd,zv(t),z(t),...
                phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
                psi_vd,psid,psi_v(t),psi(t));

%             if n >10
% 
%                 n = 0;
% 
% 
%                 [x(t),y(t),z(t),xv(t),yv(t),zv(t),phi(t),theta(t),psi(t),...
%                     phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t)] = ...
%                     quad_in_imbalance(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1(t-1),w2(t-1),w3(t-1),w4(t-1),...
%                     x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
%                     zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));


                %% PD xy
%                 [w1(t),w2(t),w3(t),w4(t)] = xypd1(m,l,k,b,Ixx,Iyy,Izz,KxD,KxP,KyD,KyP,KzD,KzP,KphiDx,KphiPx,KthetaDx,...
%                     KthetaPx,KpsiD,KpsiP,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
%                     phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

                %% PID xy
%                 [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
%                     xypid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),KxD,KxP,KxI,eI_y(t-1),KyD,KyP,KyI,...
%                     KzD,KzP,KzI,eI_z(t-1),KphiDx,KphiPx,KphiIx, ...
%                     eI_phi(t-1),KthetaDx,KthetaPx,KthetaIx,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
%                     max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
%                     phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

                %% FSOM xy
%                 [w1(t),w2(t),w3(t),w4(t)] = xyfsom1(m,l,k,b,Ixx,Iyy,Izz,Kx,Ky,Kz,Kphi_xy,Ktheta_xy,...
%                     Kpsi,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
%                     phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

                %% BSC xy
%                 [w1(t),w2(t),w3(t),w4(t)] = xybsc1(m,l,k,b,Ixx,Iyy,Izz,Kx1,Kx2,Ky1,Ky2,Kz1,Kz2,Kphi1_xy, ...
%                     Kphi2_xy,Ktheta1_xy,Ktheta2_xy,Kpsi1,Kpsi2,max_w,xvd,xd(t),xv(t),x(t),x(t-1),yvd,yd(t),yv(t),y(t),y(t-1),zvd,zd,zv(t),z(t),z(t-1),...
%                     phi_vd,phi_v(t),phi(t),phi(t-1),theta_vd,theta_v(t),theta(t),theta(t-1),psi_vd,psid,psi_v(t),psi(t),psi(t-1));

                %% SMC xy            
%                 [w1(t),w2(t),w3(t),w4(t)] = xysmc1(m,l,k,b,Ixx,Iyy,Izz,Kx,Kx1,Kx2,Ky,Ky1,Ky2, Kz,Kz1,Kz2, Kphi,Kphi1_xy, ...
%                     Kphi2_xy, Ktheta,Ktheta1_xy,Ktheta2_xy,Kpsi,Kpsi1,Kpsi2,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
%                     phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


%             end

        end

        figure(addnoise);
        subplot(4,1,1);
        qpl = plot(tspan,w1(1:timepoints), tspan,w2(1:timepoints), tspan, w3(1:timepoints), tspan, w4(1:timepoints));
        ax = gca;
        ax.FontSize = 14;
        legend('w1','w1','w3','w4','Location','northwest');
        title('Motor Speed',FontSize=22);
        ylabel('Motor Angular Velocity (Deg/s)');
        xlabel('Time (s)');
        axis([35 50 0 2000]);
        grid;

        subplot(4,1,2);
        %plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd,tspan , zd);
        ppl = plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd);
        ppl(1).LineWidth = 2;
        ppl(2).LineWidth = 2;
        ppl(3).LineWidth = 2;
        ppl(4).LineWidth = 2;
        ax = gca;
        ax.FontSize = 14;
        legend('x','y','z','desired xy','Location','northwest');
        title('Translational Motion',FontSize=22);
        ylabel('Movement (m)');
        xlabel('Time (s)');
        axis([35 50 -5 25]);
        grid;


        subplot(4,1,3);
        rpl =plot(tspan, phi*180/pi,':',tspan, theta*180/pi,'--', tspan, psi*180/pi);
        rpl(1).LineWidth = 1;
        rpl(2).LineWidth = 1;
        rpl(3).LineWidth = 1;
        ax = gca;
        ax.FontSize = 14;
        legend('phi','theta','psi','Location','northwest');
        title('Angle Motion','FontSize',22);
        ylabel('Angle (Deg)');
        xlabel('Time (s)');
        axis([35 50 -80 80]);
        grid;


        subplot(4,1,4);
        spl = plot(tspan, phi_v*180/pi,':',tspan, theta_v*180/pi,'--', tspan, psi_v*180/pi);
        spl(1).LineWidth = 1;
        spl(2).LineWidth = 1;
        spl(3).LineWidth = 1;
        ax = gca;
        ax.FontSize = 14;
        legend('phi velocity','theta velocity','psi velocity','Location','northwest');
        title('Angular Velocity','FontSize',22);
        ylabel('Angular Velocity (Deg/s');
        xlabel('Time (s)');
        axis([35 50 -400 400]);
        grid;


        if (addnoise == 1)
            [OS_x1,ts_x1,tr_x1,trl_x1,tp_x1,T_x1,U_x1,Y_x1,index_x1] = StepResponse(tspan(501:2500),xd(501:2500),x(501:2500),10^-6);
            [OS_x2,ts_x2,tr_x2,trl_x2,tp_x2,T_x2,U_x2,Y_x2,index_x2] = StepResponse(tspan(2001:4000),xd(2001:4000)-10,x(2001:4000)-10,10^-6);
            [OS_y1,ts_y1,tr_y1,trl_y1,tp_y1,T_y1,U_y1,Y_y1,index_y1] = StepResponse(tspan(501:2500),yd(501:2500),y(501:2500),10^-6);
            [OS_y2,ts_y2,tr_y2,trl_y2,tp_y2,T_y2,U_y2,Y_y2,index_y2] = StepResponse(tspan(2001:4000),yd(2001:4000)-10,y(2001:4000)-10,10^-6);
            [OS_z,ts_z,tr_z,trl_z,tp_z,T_z,U_z,Y_z,index_z] = StepResponse(tspan(1:2000),zdx(1:2000),z(1:2000),10^-6);

            phi_data = [max(phi(5:500)),min(phi(5:500)),mean(phi(5:500)),std(phi(5:500)), ...
                max(phi(1001:1500)),min(phi(1001:1500)),mean(phi(1001:1500)),std(phi(1001:1500)), ...
                max(phi(2501:3000)),min(phi(2501:3000)),mean(phi(2501:3000)),std(phi(2501:3000))];
            phi_v_data = [max(phi_v(5:500)),min(phi_v(5:500)),mean(phi_v(5:500)),std(phi_v(5:500)), ...
                max(phi_v(1001:1500)),min(phi_v(1001:1500)),mean(phi_v(1001:1500)),std(phi_v(1001:1500)), ...
                max(phi_v(2501:3000)),min(phi_v(2501:3000)),mean(phi_v(2501:3000)),std(phi_v(2501:3000))];

            theta_data = [max(theta(5:500)),min(theta(5:500)),mean(theta(5:500)),std(theta(5:500)), ...
                max(theta(1001:1500)),min(theta(1001:1500)),mean(theta(1001:1500)),std(theta(1001:1500)), ...
                max(theta(2501:3000)),min(theta(2501:3000)),mean(theta(2501:3000)),std(theta(2501:3000))];
            theta_v_data = [max(theta_v(5:500)),min(theta_v(5:500)),mean(theta_v(5:500)),std(theta_v(5:500)), ...
                max(theta_v(1001:1500)),min(theta_v(1001:1500)),mean(theta_v(1001:1500)),std(theta_v(1001:1500)), ...
                max(theta_v(2501:3000)),min(theta_v(2501:3000)),mean(theta_v(2501:3000)),std(theta_v(2501:3000))];

            psi_data = [max(psi(5:500)),min(psi(5:500)),mean(psi(5:500)),std(psi(5:500)), ...
                max(psi(1001:1500)),min(psi(1001:1500)),mean(psi(1001:1500)),std(psi(1001:1500)), ...
                max(psi(2501:3000)),min(psi(2501:3000)),mean(psi(2501:3000)),std(psi(2501:3000))];
            psi_v_data = [max(psi_v(5:500)),min(psi_v(5:500)),mean(psi_v(5:500)),std(psi_v(5:500)), ...
                max(psi_v(1001:1500)),min(psi_v(1001:1500)),mean(psi_v(1001:1500)),std(psi_v(1001:1500)), ...
                max(psi_v(2501:3000)),min(psi_v(2501:3000)),mean(psi_v(2501:3000)),std(psi_v(2501:3000))];

            w1_data = [max(w1(5:500)),min(w1(5:500)),mean(w1(5:500)),std(w1(5:500)), ...
                max(w1(1001:1500)),min(w1(1001:1500)),mean(w1(1001:1500)),std(w1(1001:1500)), ...
                max(w1(2501:3000)),min(w1(2501:3000)),mean(w1(2501:3000)),std(w1(2501:3000))];

            w2_data = [max(w2(5:500)),min(w2(5:500)),mean(w2(5:500)),std(w2(5:500)), ...
                max(w2(1001:1500)),min(w2(1001:1500)),mean(w2(1001:1500)),std(w2(1001:1500)), ...
                max(w2(2501:3000)),min(w2(2501:3000)),mean(w2(2501:3000)),std(w2(2501:3000))];

            w3_data = [max(w3(5:500)),min(w3(5:500)),mean(w3(5:500)),std(w3(5:500)), ...
                max(w3(1001:1500)),min(w3(1001:1500)),mean(w3(1001:1500)),std(w3(1001:1500)), ...
                max(w3(2501:3000)),min(w3(2501:3000)),mean(w3(2501:3000)),std(w3(2501:3000))];

            w4_data = [max(w4(5:500)),min(w4(5:500)),mean(w4(5:500)),std(w4(5:500)), ...
                max(w4(1001:1500)),min(w4(1001:1500)),mean(w4(1001:1500)),std(w4(1001:1500)), ...
                max(w4(2501:3000)),min(w4(2501:3000)),mean(w4(2501:3000)),std(w4(2501:3000))];
        end
        if (addnoise == 2)
            x_noise_data = [max(x(501:1000)),min(x(501:1000)),mean(x(501:1000)),std(x(501:1000)), ...
                max(x(1501:2500)),min(x(1501:2500)),mean(x(1501:2500)),std(x(1501:2500)), ...
                max(x(3001:4000)),min(x(3001:4000)),mean(x(3001:4000)),std(x(3001:4000))];

            y_noise_data = [max(y(501:1000)),min(y(501:1000)),mean(y(501:1000)),std(y(501:1000)), ...
                max(y(1501:2500)),min(y(1501:2500)),mean(y(1501:2500)),std(y(1501:2500)), ...
                max(y(3001:4000)),min(y(3001:4000)),mean(y(3001:4000)),std(y(3001:4000))];
            
            z_noise_data = [max(z(501:1000)),min(z(501:1000)),mean(z(501:1000)),std(z(501:1000)), ...
                max(z(1501:2500)),min(z(1501:2500)),mean(z(1501:2500)),std(z(1501:2500)), ...
                max(z(3001:4000)),min(z(3001:4000)),mean(z(3001:4000)),std(z(3001:4000))];

            phi_noise_data = [max(phi(501:1000)),min(phi(501:1000)),mean(phi(501:1000)),std(phi(501:1000)), ...
                max(phi(1501:2500)),min(phi(1501:2500)),mean(phi(1501:2500)),std(phi(1501:2500)), ...
                max(phi(3001:4000)),min(phi(3001:4000)),mean(phi(3001:4000)),std(phi(3001:4000))];

            theta_noise_data = [max(theta(501:1000)),min(theta(501:1000)),mean(theta(501:1000)),std(theta(501:1000)), ...
                max(theta(1501:2500)),min(theta(1501:2500)),mean(theta(1501:2500)),std(theta(1501:2500)), ...
                max(theta(3001:4000)),min(theta(3001:4000)),mean(theta(3001:4000)),std(theta(3001:4000))];

            psi_noise_data = [max(psi(501:1000)),min(psi(501:1000)),mean(psi(501:1000)),std(psi(501:1000)), ...
                max(psi(1501:2500)),min(psi(1501:2500)),mean(psi(1501:2500)),std(psi(1501:2500)), ...
                max(psi(3001:4000)),min(psi(3001:4000)),mean(psi(3001:4000)),std(psi(3001:4000))];

            phiv_noise_data = [max(phi_v(501:1000)),min(phi_v(501:1000)),mean(phi_v(501:1000)),std(phi_v(501:1000)), ...
                max(phi_v(1501:2500)),min(phi_v(1501:2500)),mean(phi_v(1501:2500)),std(phi_v(1501:2500)), ...
                max(phi_v(3001:4000)),min(phi_v(3001:4000)),mean(phi_v(3001:4000)),std(phi_v(3001:4000))];

            thetav_noise_data = [max(theta_v(501:1000)),min(theta_v(501:1000)),mean(theta_v(501:1000)),std(theta_v(501:1000)), ...
                max(theta_v(1501:2500)),min(theta_v(1501:2500)),mean(theta_v(1501:2500)),std(theta_v(1501:2500)), ...
                max(theta_v(3001:4000)),min(theta_v(3001:4000)),mean(theta_v(3001:4000)),std(theta_v(3001:4000))];

            psiv_noise_data = [max(psi_v(501:1000)),min(psi_v(501:1000)),mean(psi_v(501:1000)),std(psi_v(501:1000)), ...
                max(psi_v(1501:2500)),min(psi_v(1501:2500)),mean(psi_v(1501:2500)),std(psi_v(1501:2500)), ...
                max(psi_v(3001:4000)),min(psi_v(3001:4000)),mean(psi_v(3001:4000)),std(psi_v(3001:4000))];

            w1_noise_data = [max(w1(501:1000)),min(w1(501:1000)),mean(w1(501:1000)),std(w1(501:1000)), ...
                max(w1(1501:2500)),min(w1(1501:2500)),mean(w1(1501:2500)),std(w1(1501:2500)), ...
                max(w1(3001:4000)),min(w1(3001:4000)),mean(w1(3001:4000)),std(w1(3001:4000))];

            w2_noise_data = [max(w2(501:1000)),min(w2(501:1000)),mean(w2(501:1000)),std(w2(501:1000)), ...
                max(w2(1501:2500)),min(w2(1501:2500)),mean(w2(1501:2500)),std(w2(1501:2500)), ...
                max(w2(3001:4000)),min(w2(3001:4000)),mean(w2(3001:4000)),std(w2(3001:4000))];

            w3_noise_data = [max(w3(501:1000)),min(w3(501:1000)),mean(w3(501:1000)),std(w3(501:1000)), ...
                max(w3(1501:2500)),min(w3(1501:2500)),mean(w3(1501:2500)),std(w3(1501:2500)), ...
                max(w3(3001:4000)),min(w3(3001:4000)),mean(w3(3001:4000)),std(w3(3001:4000))];

            w4_noise_data = [max(w4(501:1000)),min(w4(501:1000)),mean(w4(501:1000)),std(w4(501:1000)), ...
                max(w4(1501:2500)),min(w4(1501:2500)),mean(w4(1501:2500)),std(w4(1501:2500)), ...
                max(w4(3001:4000)),min(w4(3001:4000)),mean(w4(3001:4000)),std(w4(3001:4000))];

        end

%         figure(addnoise);
%         subplot(4,1,1);
%         qpl = plot(tspan,w1(1:timepoints), tspan,w2(1:timepoints), tspan, w3(1:timepoints), tspan, w4(1:timepoints));
%         legend('w1','w1','w3','w4','Location','northwest');
%         title('Motor Speed')
%         ylabel('Motor Angular Velocity (Deg/s)')
%         xlabel('Time (s)')
%         pp= qq+5;
%         rr = pp+5;
%         ss = rr+5;
% 
%         subplot(4,1,2);
%         %plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd,tspan , zd);
%         ppl = plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd);
%         ppl(1).LineWidth = 2;
%         ppl(2).LineWidth = 2;
%         ppl(3).LineWidth = 2;
%         ppl(4).LineWidth = 2;
%         legend('x','y','z','desired xy','Location','northwest');
%         title('Cartesean Movement')
%         ylabel('Movement (m)')
%         xlabel('Time (s)')
%         subplot(4,1,3);
%         rpl =plot(tspan, phi*180/pi,':',tspan, theta*180/pi,'--', tspan, psi*180/pi);
%         rpl(1).LineWidth = 1;
%         rpl(2).LineWidth = 1;
%         rpl(3).LineWidth = 1;
%         legend('phi','theta','psi','Location','northwest');
%         title('Angle Movement')
%         ylabel('Angle (Deg)')
%         xlabel('Time (s)')
%         subplot(4,1,4);
%         spl = plot(tspan, phi_v*180/pi,':',tspan, theta_v*180/pi,'--', tspan, psi_v*180/pi);
%         spl(1).LineWidth = 1;
%         spl(2).LineWidth = 1;
%         spl(3).LineWidth = 1;
%         legend('phi velocity','theta velocity','psi velocity','Location','northwest');
%         title('Angular Velocity')
%         ylabel('Angular Velocity (Deg/s')
%         xlabel('Time (s)')

    end
    Type = {'PD'};
    OS_xy = round((OS_x1+OS_x2+OS_y1+OS_y2)/4,2);
    OS_z = round(OS_z,2);
    ts_xy = (ts_x1+ts_x2+ts_y1+ts_y2)/4;
    tr_xy = (tr_x1+tr_x2+tr_y1+tr_y2)/4;
    trl_xy = (trl_x1+trl_x2+trl_y1+trl_y2)/4;
    tp_xy = (tp_x1+tp_x2+tp_y1+tp_y2)/4;

    x_noise_data = round(real(x_noise_data),2);
    y_noise_data = round(real(y_noise_data),2);
    z_noise_data = round(real(z_noise_data),2); 

    xy_noise_data_std = (x_noise_data(12)+y_noise_data(12))/2;
    z_noise_data_std = z_noise_data(12);
    
    

    phi_data = round(real(phi_data*180/pi),2);
    theta_data = round(real(theta_data*180/pi),2);
    psi_data = round(real(psi_data*180/pi),2);
    phi_v_data = round(real(phi_v_data*180/pi),2);
    theta_v_data = round(real(theta_v_data*180/pi),2);
    psi_v_data = round(real(psi_v_data*180/pi),2);
    
    phi_data_max = abs(phi_data(9));
    phi_data_std = phi_data(12);
    theta_data_max = abs(theta_data(9));
    theta_data_std = theta_data(12);
    psi_data_max = abs(psi_data(9));
    psi_data_std = psi_data(12);
    phi_v_data_max = abs(phi_v_data(9));
    phi_v_data_std = phi_v_data(12);
    theta_v_data_max = abs(theta_v_data(9));
    theta_v_data_std = theta_v_data(12);
    psi_v_data_max = abs(psi_v_data(9));
    psi_v_data_std = psi_v_data(12);

    w1_data = round(real(w1_data),2);
    w2_data = round(real(w2_data),2);
    w3_data = round(real(w3_data),2);
    w4_data = round(real(w4_data),2);


    w_data_range = w4_data(9)-w4_data(10);
    w_data_std = w4_data(12);
    
    w1_noise_data = round(real(w1_noise_data),2);
    w2_noise_data = round(real(w2_noise_data),2);
    w3_noise_data = round(real(w3_noise_data),2);
    w4_noise_data = round(real(w4_noise_data),2);

    w_noise_data_range = w4_noise_data(9)-w4_noise_data(10);
    w_noise_data_std = w4_noise_data(12);
    

%     if (startloop == 0)
%         startloop = startloop+ 1;
% 
%         T_data = table(startloop,Type,...
%             KxD,KxP,KyD,KyP,KzD,KzP,KphiD,KphiP, ... 
%             KthetaD,KthetaP,KpsiD,KpsiP,...
%             KphiDx,KphiPx,KthetaDx,KthetaPx,...
%             OS_xy,ts_xy,tr_xy,trl_xy,tp_xy,...
%             OS_z,ts_z,tr_z,trl_z,tp_z,...
%             xy_noise_data_std,z_noise_data_std, ...
%             phi_data_max,phi_data_std, ...
%             theta_data_max,theta_data_std,...
%             psi_data_max,psi_data_std, ...
%             phi_v_data_max,phi_v_data_std,...
%             theta_v_data_max,theta_v_data_std, ...
%             psi_v_data_max,psi_v_data_std, ...
%             w_data_range,w_data_std,...
%             w_noise_data_range,w_noise_data_std);
% 
%             % KxD,KxP,KxI,KyD,KyP,KyI,KzD,KzP,KzI,KphiD,KphiP,KphiI, ... 
%             % KthetaD,KthetaP,KthetaI,KpsiD,KpsiP,KpsiI,...  
%             % Kx,Ky,Kz,Kphi,Ktheta,Kpsi,...
%             % Kx1,Kx2,Ky1,Ky2,Kz1,Kz2,...
%             % Kphi1,Kphi2,Ktheta1,Ktheta2,Kpsi1,Kpsi2,...
%             % Kx,Kx1,Kx2,Ky,Ky1,Ky2,Kz,Kz1,Kz2,...
%             % Kphi,Kphi1,Kphi2,Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,...
%             % Kphi1_xy,Kphi2_xy,Ktheta1_xy,Ktheta2_xy,...
% 
%         writetable(T_data, 'myData1.csv','WriteRowNames',true);
%     else
%         startloop = startloop + 1;
% 
%         T_data = table(startloop,Type,...
%             KxD,KxP,KyD,KyP,KzD,KzP,KphiD,KphiP, ... 
%             KthetaD,KthetaP,KpsiD,KpsiP,...
%             KphiDx,KphiPx,KthetaDx,KthetaPx,...
%             OS_xy,ts_xy,tr_xy,trl_xy,tp_xy,...
%             OS_z,ts_z,tr_z,trl_z,tp_z,...
%             xy_noise_data_std,z_noise_data_std, ...
%             phi_data_max,phi_data_std, ...
%             theta_data_max,theta_data_std,...
%             psi_data_max,psi_data_std, ...
%             phi_v_data_max,phi_v_data_std,...
%             theta_v_data_max,theta_v_data_std, ...
%             psi_v_data_max,psi_v_data_std, ...
%             w_data_range,w_data_std,...
%             w_noise_data_range,w_noise_data_std);
% 
%         writetable(T_data, 'myData1.csv','WriteMode','Append',...
%             'WriteVariableNames',false,'WriteRowNames',true);
%     end   

end
end
end
end

% figure(1)
% plot(tspan,w1(1:timepoints), tspan,w2(1:timepoints), tspan, w3(1:timepoints), tspan, w4(1:timepoints));
%
% figure(2)
% plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd,tspan , zd);
%
% figure(3)
% plot(tspan, phi*180/pi,':',tspan, theta*180/pi,'--', tspan, psi*180/pi);
%
% figure(4)
% plot(tspan, phi_v*180/pi,':',tspan, theta_v*180/pi,'--', tspan, psi_v*180/pi) ;

function [x,y,z,xv,yv,zv,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r] = ...
    quad_in(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1,w2,w3,w4,xin,yin,zin,xvin,yvin,...
    zvin,phi_in,theta_in,psi_in,pin,qin,rin)

g = 9.81;

T = k*(w1*w1+w2*w2+w3*w3+w4*w4);
x_a = T/m *(cos(psi_in)*sin(theta_in))*cos(phi_in)+ ...
    sin(phi_in)*sin(psi_in)-1/m*Ax*xvin;
y_a = T/m *(sin(psi_in)*sin(theta_in)*cos(phi_in)- ...
    sin(phi_in)*cos(psi_in))-1/m*Ay*yvin;
z_a = -g+T/m*cos(phi_in)*cos(theta_in)-1/m*Az*zvin;

p_a = (Iyy-Izz)/Ixx*qin*rin- ...
    Ir/Ixx*qin*(+w1-w2+w3-w4) + ...
    k*l/Ixx*(w4*w4-w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(+w1-w2+w3-w4)+...
    k*l/Iyy*(w3*w3-w1*w1);

r_a = b/Izz*(w1*w1-w2*w2+w3*w3-w4*w4);

xv = xvin+x_a*tdel;
yv = yvin+y_a*tdel;
zv = zvin+z_a*tdel;

p = pin+p_a*tdel;
q = qin+q_a*tdel;
r = rin+r_a*tdel;

x = xin+xvin*tdel;
y = yin+yvin*tdel;
z = zin+zvin*tdel;

phi_v = pin+(qin*sin(phi_in)+rin*cos(phi_in))*tan(theta_in);
theta_v = qin*cos(phi_in)-rin*sin(phi_in);
psi_v = (qin*sin(phi_in)+rin*cos(phi_in))/cos(theta_in);

phi = phi_in+phi_v*tdel;
if phi > pi
    phi = phi- 2*pi;
elseif    phi < -pi
    phi = phi+ 2*pi;
end

theta = theta_in+theta_v*tdel;
if theta > pi
    theta = theta- 2*pi;
elseif    theta < -pi
    theta = theta+ 2*pi;
end


psi = psi_in+psi_v*tdel;
if psi > pi
    psi = psi- 2*pi;
elseif    psi < -pi
    psi = psi+ 2*pi;
end

end

function [x,y,z,xv,yv,zv,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r] = ...
    quad_in_imbalance(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1,w2,w3,w4,xin,yin,zin,xvin,yvin,...
    zvin,phi_in,theta_in,psi_in,pin,qin,rin)

g = 9.81;
imb1 =.94;
imb2 = 1.06;

T = k*(imb1*w1*imb1*w1+imb2*imb2*w2*w2+w3*w3+w4*w4);
x_a = T/m *(cos(psi_in)*sin(theta_in))*cos(phi_in)+ ...
    sin(phi_in)*sin(psi_in)-1/m*Ax*xvin;
y_a = T/m *(sin(psi_in)*sin(theta_in)*cos(phi_in)- ...
    sin(phi_in)*cos(psi_in))-1/m*Ay*yvin;
z_a = -g+T/m*cos(phi_in)*cos(theta_in)-1/m*Az*zvin;

p_a = (Iyy-Izz)/Ixx*qin*rin- ...
    Ir/Ixx*qin*(+imb1*w1-imb2*w2+w3-w4) + ...
    k*l/Ixx*(w4*w4-imb2*imb2*w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(+imb1*w1-imb2*w2+w3-w4)+...
    k*l/Iyy*(w3*w3-imb1*w1*imb1*w1);

r_a = b/Izz*(imb1*w1*imb1*w1-imb2*imb2*w2*w2+w3*w3-w4*w4);

xv = xvin+x_a*tdel;
yv = yvin+y_a*tdel;
zv = zvin+z_a*tdel;

p = pin+p_a*tdel;
q = qin+q_a*tdel;
r = rin+r_a*tdel;

x = xin+xvin*tdel;
y = yin+yvin*tdel;
z = zin+zvin*tdel;

phi_v = pin+(qin*sin(phi_in)+rin*cos(phi_in))*tan(theta_in);
theta_v = qin*cos(phi_in)-rin*sin(phi_in);
psi_v = (qin*sin(phi_in)+rin*cos(phi_in))/cos(theta_in);

phi = phi_in+phi_v*tdel;
if phi > pi
    phi = phi- 2*pi;
elseif    phi < -pi
    phi = phi+ 2*pi;
end

theta = theta_in+theta_v*tdel;
if theta > pi
    theta = theta- 2*pi;
elseif    theta < -pi
    theta = theta+ 2*pi;
end


psi = psi_in+psi_v*tdel;
if psi > pi
    psi = psi- 2*pi;
elseif    psi < -pi
    psi = psi+ 2*pi;
end

end


function [w1,w2,w3,w4] = pd1(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z))*m/cos(phi)/cos(theta);
torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi))*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta))*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi))*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = xypd1(m,l,k,b,Ixx,Iyy,Izz,KxD,KxP,KyD,KyP,KzD,KzP,KphiD,KphiP,KthetaD,...
    KthetaP,KpsiD,KpsiP,max_w,xvd,xd,xv,x,yvd,yd,yv,y,zvd,zd,zv,z,...
    phivd,phiv,phi,thetavd,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z))*m/cos(phi)/cos(theta);

thetad = m/T_PD *(KxD*(xvd - xv) + KxP*(xd - x));
phid = - m/T_PD *(KyD*(yvd - yv) + KyP*(yd - y));

torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi))*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta))*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi))*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);



if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >1000
    w3 = 1000;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4,eI_x_out,eI_y_out,eI_z_out,eI_phi_out,eI_theta_out,eI_psi_out] = ...
    pid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x,eI_y,...
    KzD,KzP,KzI,eI_z,KphiD,KphiP,KphiI, ...
    eI_phi,KthetaD,KthetaP,KthetaI,eI_theta,KpsiD,KpsiP,KpsiI,eI_psi,...
    max_w,xd,x,yd,y,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

eI_x_out = eI_x+(xd - x)*tdel;
eI_y_out = eI_y+(yd - y)*tdel;
eI_z_out = eI_z+(zd - z)*tdel;
eI_phi_out = eI_phi+(phid - phi)*tdel;
eI_theta_out = eI_theta+(thetad - theta)*tdel;
eI_psi_out = eI_psi+(psid - psi)*tdel;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z)+ KzI*eI_z_out)*m/cos(phi)/cos(theta);
torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi)+KphiI*eI_phi_out)*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta)+KthetaI*eI_theta_out)*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi)+KpsiI*eI_psi_out)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4,eI_x_out,eI_y_out,eI_z_out,eI_phi_out,eI_theta_out,eI_psi_out] = ...
    xypid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x,KxD,KxP,KxI,eI_y,KyD,KyP,KyI,...
    KzD,KzP,KzI,eI_z,KphiD,KphiP,KphiI, ...
    eI_phi,KthetaD,KthetaP,KthetaI,eI_theta,KpsiD,KpsiP,KpsiI,eI_psi,...
    max_w,xvd,xd,xv,x,yvd,yd,yv,y,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

eI_x_out = eI_x+(xd - x)*tdel;
eI_y_out = eI_y+(yd - y)*tdel;
eI_z_out = eI_z+(zd - z)*tdel;

eI_phi_out = eI_phi+(phid - phi)*tdel;
eI_theta_out = eI_theta+(thetad - theta)*tdel;
eI_psi_out = eI_psi+(psid - psi)*tdel;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z)+ KzI*eI_z_out)*m/cos(phi)/cos(theta);

thetad = m/T_PD *(KxD*(xvd - xv) + KxP*(xd - x)+ KxI*eI_x_out );
phid = - m/T_PD *(KyD*(yvd - yv) + KyP*(yd - y)+ KyI*eI_y_out);

torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi)+KphiI*eI_phi_out)*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta)+KthetaI*eI_theta_out)*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi)+KpsiI*eI_psi_out)*Izz;


w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);



if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >1000
    w3 = 1000;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = fsom1(m,l,k,b,Ixx,Iyy,Izz,Kz, Kphi,Ktheta,Kpsi,max_w,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;


sz = (zvd - zv)+(zd - z);
sphi = (phivd - phiv)+(phid - phi);
stheta = (thetavd - thetav)+(thetad - theta);
spsi = (psivd - psiv)+(psid - psi);

T_PD = (g + Kz*sqrt(abs(sz ))*sign(sz))*m/cos(phi)/cos(theta);
torq_phi_PD = Kphi*sqrt(abs(sphi ))*sign(sphi)*Ixx;
torq_theta_PD = Ktheta*sqrt(abs(stheta ))*sign(stheta)*Iyy;
torq_psi_PD = Kpsi*sqrt(abs(spsi ))*sign(spsi)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = xyfsom1(m,l,k,b,Ixx,Iyy,Izz,Kx,Ky,Kz, Kphi,Ktheta,...
    Kpsi,max_w,xvd,xd,xv,x,yvd,yd,yv,y,zvd,zd,zv,z,...
    phivd,phiv,phi,thetavd,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

sz = (zvd - zv)+(zd - z);
sx = (xvd - xv)+(xd - x);
sy = (yvd - yv)+(yd - y);

T_PD = (g + Kz*sqrt(abs(sz ))*sign(sz))*m/cos(phi)/cos(theta);

thetad = m/T_PD *Kx*sqrt(abs(sx ))*sign(sx);
phid = - m/T_PD *Ky*sqrt(abs(sy ))*sign(sy);

sphi = (phivd - phiv)+(phid - phi);
stheta = (thetavd - thetav)+(thetad - theta);
spsi = (psivd - psiv)+(psid - psi);

torq_phi_PD = Kphi*sqrt(abs(sphi ))*sign(sphi)*Ixx;
torq_theta_PD = Ktheta*sqrt(abs(stheta ))*sign(stheta)*Iyy;
torq_psi_PD = Kpsi*sqrt(abs(spsi ))*sign(spsi)*Izz;


w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);



if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >1000
    w3 = 1000;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = bsc1(m,l,k,b,Ixx,Iyy,Izz,Kz1,Kz2, Kphi1, ...
    Kphi2, Ktheta1,Ktheta2,Kpsi1,Kpsi2,max_w,zvd,zd,zv,z,z_b,...
    phivd,phid,phiv,phi,phi_b,thetavd,thetad,thetav,theta,theta_b,psivd,psid,psiv,psi,psi_b)

g = 9.81;


sz = -(zvd - zv)-Kz1*(zd - z);
sphi = -(phivd - phiv)-Kphi1*(phid - phi);
stheta = -(thetavd - thetav)-Ktheta1*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi1*(psid - psi);

T_PD = (g +((zd - z)-Kz1*(sz +Kz1*(zd - z_b))-Kz2*sz))*m/cos(phi)/cos(theta);
torq_phi_PD = ((phid - phi)-Kphi1*(sphi +Kphi1*(phid - phi_b))-Kphi2*sphi)*Ixx;
torq_theta_PD = ((thetad - theta)-Ktheta1*(stheta +Ktheta1*(thetad - theta_b))-Ktheta2*stheta)*Iyy;
torq_psi_PD = ((psid - psi)-Kpsi1*(spsi +Kpsi1*(psid - psi_b))-Kpsi2*spsi)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = xybsc1(m,l,k,b,Ixx,Iyy,Izz,Kx1,Kx2,Ky1,Ky2,Kz1,Kz2, Kphi1, ...
    Kphi2, Ktheta1,Ktheta2,Kpsi1,Kpsi2,max_w,xvd,xd,xv,x,x_b,yvd,yd,yv,y,y_b,zvd,zd,zv,z,z_b,...
    phivd,phiv,phi,phi_b,thetavd,thetav,theta,theta_b,psivd,psid,psiv,psi,psi_b)

g = 9.81;

% phid = 0;
% thetad =0;

sz = -(zvd - zv)-Kz1*(zd - z);
sx = -(xvd - xv)-Kx1*(xd - x);
sy = -(yvd - yv)-Ky1*(yd - y);

T_PD = (g +((zd - z)-Kz1*(sz +Kz1*(zd - z_b))-Kz2*sz))*m/cos(phi)/cos(theta);

thetad = m/T_PD *((xd - x)-Kx1*(sx +Kx1*(xd - x_b))-Kx2*sx);
phid = - m/T_PD *((yd - y)-Ky1*(sy +Ky1*(yd - y_b))-Ky2*sy);

sphi = -(phivd - phiv)-Kphi1*(phid - phi);
stheta = -(thetavd - thetav)-Ktheta1*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi1*(psid - psi);


torq_phi_PD = ((phid - phi)-Kphi1*(sphi +Kphi1*(phid - phi_b))-Kphi2*sphi)*Ixx;
torq_theta_PD = ((thetad - theta)-Ktheta1*(stheta +Ktheta1*(thetad - theta_b))-Ktheta2*stheta)*Iyy;
torq_psi_PD = ((psid - psi)-Kpsi1*(spsi +Kpsi1*(psid - psi_b))-Kpsi2*spsi)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end


end

function [w1,w2,w3,w4] = smc1(m,l,k,b,Ixx,Iyy,Izz,Kz,Kz1,Kz2, Kphi,Kphi1, ...
    Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;


e_phiv = (phivd - phiv)
e_phi = (phid - phi)

sz = -(zvd - zv)-Kz*(zd - z);
sphi = -(phivd - phiv)-Kphi*(phid - phi)
stheta = -(thetavd - thetav)-Ktheta*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi*(psid - psi);



T_PD = (g +(-Kz*Kz*(zd - z)-Kz1*sign(sz)-Kz2*sz))*m/cos(phi)/cos(theta)



torq_phi_PD = (-Kphi*Kphi*(phid - phi)-Kphi1*sign(sphi)-Kphi2*sphi)*Ixx

torq_phi_PD1 = -Kphi*Kphi*(phid - phi)
torq_phi_PD2 = -Kphi1*sign(sphi)
torq_phi_PD3 = -Kphi2*sphi


torq_theta_PD = (-Ktheta*Ktheta*(thetad - theta)-Ktheta1*sign(stheta)-Ktheta2*stheta)*Iyy;
torq_psi_PD = (-Kpsi*Kpsi*(psid - psi)-Kpsi1*sign(spsi)-Kpsi2*spsi)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end

end

function [w1,w2,w3,w4] = xysmc1(m,l,k,b,Ixx,Iyy,Izz,Kx,Kx1,Kx2,Ky,Ky1,Ky2, Kz,Kz1,Kz2, Kphi,Kphi1, ...
    Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,xvd,xd,xv,x,yvd,yd,yv,y,zvd,zd,zv,z,...
    phivd,phiv,phi,thetavd,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

% phid = 0;
% thetad =0;

sz = -(zvd - zv)-Kz*(zd - z);
sx = -(xvd - xv)-Kx*(xd - x);
sy = -(yvd - yv)-Ky*(yd - y);

T_PD = (g +(-Kz*Kz*(zd - z)-Kz1*sign(sz)-Kz2*sz))*m/cos(phi)/cos(theta);

thetad = m/T_PD *(-Kx*Kx*(xd - x)-Kx1*erf(real(sx))-Kx2*sx);
phid = - m/T_PD *(-Ky*Ky*(yd - y)-Ky1*erf(real(sy))-Ky2*sy);


sphi = -(phivd - phiv)-Kphi*(phid - phi);
stheta = -(thetavd - thetav)-Ktheta*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi*(psid - psi);


torq_phi_PD = (-Kphi*Kphi*(phid - phi)-Kphi1*sign(sphi)-Kphi2*sphi)*Ixx;
torq_theta_PD = (-Ktheta*Ktheta*(thetad - theta)-Ktheta1*sign(stheta)-Ktheta2*stheta)*Iyy;
torq_psi_PD = (-Kpsi*Kpsi*(psid - psi)-Kpsi1*sign(spsi)-Kpsi2*spsi)*Izz;

w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

if w1 >max_w
    w1 = max_w;
elseif w1 < 0
    w1 =0;
end
if w2 >max_w
    w2 = max_w;
elseif w2 < 0
    w2 =0;
end
if w3 >max_w
    w3 = max_w;
elseif w3 < 0
    w3 =0;
end
if w4 >max_w
    w4 = max_w;
elseif w4 < 0
    w4 =0;
end


end

function [OS,ts,tr,trl,tp,T,U,Y,index] = StepResponse(t,u,y,tol)

if (nargin < 4) || (nargin > 4)
    error('StepResponse must have 4 arguments only')
end
% verify input argments are vectors
if (min(size(t)) > 1) || (min(size(u)) > 1) || (min(size(y)) > 1)
    error('StepResponse arguments t, u, and y must be vectors, not matrices')
end
if (max(size(t)) < 2) || (max(size(u)) < 2) || (max(size(y)) < 2)
    error('StepResponse arguments t, u, and y must be vectors, not scalars')
end
% verify input argument tol is a scalar
if (length(tol) > 1)
    error('StepResponse arguments tol must be a scalars')
end
% verify input arguments are the same length
if (length(t) > length(u)+tol) || (length(t) < length(u)-tol) || (length(t) > length(y)+tol) || (length(t) < length(y)-tol)
    error('StepResponse arguments t, u, and y must be vectors of the same length')
end

% find time when step command was given
t0 = 0; % initialize t0, the vector index when the step command is given
for J = 1:length(t)
    % search for change in u.  the 1st change above the tol is assumed
    % to be the step command
    if (t0 == 0) && (abs(u(J)) >= tol)
        t0 = J;
    end
    % continue to search for changes in u.  if additional changes
    % greater than tol are found, generate an error
    if (t0 > 0) && ((abs(u(J)) > abs(u(t0)) + tol) || (abs(u(J)) < abs(u(t0)) - tol))
        error('StepResponse must have only 1 step command.  The step command must start at 0units and move to +/- step size in 1 time sample.')
    end
end
% truncate data series to t0 and above.  zero the t vector
if (t0 == 0)
    t0 = 1;
end
% truncate data series to t0 and above.  zero the t vector
T = t(t0:length(t))-t(t0);
U = u(t0:length(t));
Y = y(t0:length(t));
% determine step size, ts tolerance, and tr points
StepSize = abs(U(1)); % step command size
Ts_utol = 1.02*StepSize; % upper ts tolerance
Ts_ltol = 0.98*StepSize; % lower ts tolerance
Tr_u = 0.9*StepSize; % upper tr point
Tr_l = 0.1*StepSize; % lower tr point
% initialize index
index = zeros(4,1);

% calculate OS and tp
[Ypeak,Ipeak] = max(abs(Y)); %  find max response value and index
tp = T(Ipeak); % tp is time at max response
index(4) = Ipeak; % tp index
if (Ypeak >= StepSize) % if max response is greater than StepSize then calculate OS
    OS = 100*(Ypeak/StepSize)-100;
else % if max response is less than StepSize then OS = 0%
    OS = 0;
end

% determine ts and tr
ts = 0; % initialize ts
Tr_ucount = 0; % initialize tr counters
Tr_lcount = 0;
% search response vector
for J = length(T):-1:1
    % search for when Y is no longer within 2% of StepSize
    if (ts == 0) && ((abs(Y(J)) > Ts_utol) || (abs(Y(J)) < Ts_ltol))
        ts = T(J);
        index(1) = J;
    end
    % search for when Y is ~90% of U
    if (Tr_ucount == 0) && (abs(Y(J)) <= Tr_u) && (T(J) < T(Ipeak))
        Tr_ucount = J;
    end
    % search for when Y is ~10% of U
    if (Tr_lcount == 0) && (abs(Y(J)) <= Tr_l) && (T(J) < T(Ipeak))
        Tr_lcount = J;
    end
end
% error check ts - ts should have been set during loop and Y should
% settle out before end of data
if (ts == 0) || (ts >= max(T)-tol)
    ts = NaN;
    index(1) = NaN;
end

if (Tr_ucount == 0)
    Tr_ucount = 1;
end
if (Tr_lcount == 0)
    Tr_lcount = 1;
end
% calculate tr and store index values
tr = round(T(Tr_ucount)-T(Tr_lcount),2);
trl = round(T(Tr_ucount),2);
index(2) = Tr_ucount;
index(3) = Tr_lcount;
% error check tr - tr should have been set during loop and tr should be
% positive
if tr <= 0
    tr = NaN;
    trl = NaN;
    index(2) = NaN;
    index(3) = NaN;
end

end

