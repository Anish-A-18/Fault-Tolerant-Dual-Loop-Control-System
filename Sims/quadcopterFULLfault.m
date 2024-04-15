


clear;
%rng(14);
rng(5)

rng(5)
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
timepoints = 1000*4+1;
tspan = linspace(0,timeend,timepoints);
tdel = timeend/(timepoints-1);

f_dist = 1;
x_amp_dist = 0.01;
phi_amp_dist = 0.01;
pi = 3.14;

xv = zeros(1,timepoints);
yv = zeros(1,timepoints);
zv = zeros(1,timepoints);

xa = zeros(1,timepoints);
ya = zeros(1,timepoints);
za = zeros(1,timepoints);

q = zeros(1,timepoints);
p = zeros(1,timepoints);
r = zeros(1,timepoints);

qa = zeros(1,timepoints);
pa = zeros(1,timepoints);
ra = zeros(1,timepoints);

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
xd(1:1000) = 0;
xd(1001:2500) = 15;
%xd(4001:6000) = 10;
xd(2501:4001) = 35;
yd(1:1000) = 0;
yd(1001:2500) = 15;
%yd(4001:6000) = 10;
yd(2501:4001) = 35;
zd = 5;
zdx(1:3801) = 5;
zdx(3801:4001) = 0;
%


N1(1:1500) = 1;
N1(1501:1511) = .6;
N1(1512:4001) = 1;

N2(1:4001) = 1;
N3(1:4001) = 1;
N4(1:4001) = 1;

w1 = ones(1,timepoints);
w2 = ones(1,timepoints);
w3 = ones(1,timepoints);
w4 = ones(1,timepoints);

w1obs = ones(1,timepoints);
w2obs = ones(1,timepoints);
w3obs = ones(1,timepoints);
w4obs = ones(1,timepoints);

w1eff = ones(1,timepoints);
w2eff = ones(1,timepoints);
w3eff = ones(1,timepoints);
w4eff = ones(1,timepoints);

eI_x = zeros(1,timepoints);
eI_y = zeros(1,timepoints);
eI_z = zeros(1,timepoints);
eI_phi = zeros(1,timepoints);
eI_theta = zeros(1,timepoints);
eI_psi = zeros(1,timepoints);

max_w = 2000;

xm = zeros(5,timepoints);
ym = zeros(5,timepoints);
zm = zeros(5,timepoints);


% linear dynamics
startloop = 1;


% bsc
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

% pid
KxD =4; %6; %4;
KxP = 4;
KxI = 1; %0; %.5;
KyD = 4;
KyP = 4;
KyI = 1; %0; %.5;
KzD = 4;
KzP = 5;
KzI = 1; %0; %.5;
KphiD = 4;
KphiP = 8;
KphiI = 4; %0; %.5;
KthetaD = 4;
KthetaP = 8;
KthetaI = 4; % %0; % .5;
KpsiD = 6;
KpsiP = 8;
KpsiI = 4; %0; %.5;
KphiDx = 4;
KphiPx = 8;
KphiIx = 4; %0; %.5;
KthetaDx = 4;
KthetaPx = 8;
KthetaIx = 4; % %0; % .5;

% Kz = 20;
% Kx = 50;
% Ky = 50;
% Kphi = 10;
% Ktheta = 10;
% Kpsi = 10;

%smc
Kz = 2; %2
Kz1 = 1;
Kz2 = 5;
Kx = 1;
Kx1 = 5;
Kx2 = 2; %1
Ky = 1;
Ky1 = 5;
Ky2 = 2; %1
Kphi = 2;
Kphi1 = 6;
Kphi2 =2;%2; %.5
Ktheta = 2;
Ktheta1 = 6;
Ktheta2 = 2; %2; %.5
Kphi1_xy = 30; %15
Kphi2_xy = 30;
Ktheta1_xy = 30; %15
Ktheta2_xy = 30;
Kpsi = 1;
Kpsi1 = 1;
Kpsi2 = 3;

NP = 0;
maxn =10;
for addnoise = 2:2
    n = 0;
    for t = 2:4001


        if t > 500
            NP = 1;
        end

        n = n+1 ;


         [w1obs(t-1),w2obs(t-1),w3obs(t-1),w4obs(t-1),w1eff(t-1),w2eff(t-1),w3eff(t-1),w4eff(t-1)] = ...
                    w_obs(m,l,k,b,Ixx,Iyy,Izz,w1(t-1),w2(t-1),w3(t-1),w4(t-1),...
                    phi(t-1),theta(t-1),za(t-1),pa(t-1),qa(t-1),ra(t-1));
         if t> 1501 & t<= 3000
             w1eff(t-1) = 1/.7*w1(t-1);
         elseif t >3000

             w1eff(t-1) = max_w;
             w2eff(t-1) = w2(t-1);
             w3eff(t-1) = w3(t-1);
             w4eff(t-1) = w4(t-1);
         end


            

        if NP == 0
             
            if t <4
                [x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),phi(t),theta(t),psi(t),...
                    phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t),pa(t),qa(t),ra(t)] = ...
                    quad_in(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,N1(t-1)*w1(t-1),N2(t-1)*w2(t-1),N3(t-1)*w3(t-1),N4(t-1)*w4(t-1),...
                    x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
                    zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));
            else
                [x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),phi(t),theta(t),psi(t),...
                    phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t),pa(t),qa(t),ra(t)] = ...
                    quad_in(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,N1(t-1)*w1(t-1),N2(t-1)*w2(t-1),N3(t-1)*w3(t-1),N4(t-1)*w4(t-1),...
                    x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
                    zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));
            end
             

        elseif NP ==1 
           
            [x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),phi(t),theta(t),psi(t),...
            phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t),pa(t),qa(t),ra(t)] = ...
            quad_in_np(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,N1(t-1)*w1(t-1),N2(t-1)*w2(t-1),N3(t-1)*w3(t-1),N4(t-1)*w4(t-1),...
            x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
            zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));
        end
        
        if addnoise == 2
            %% Disturbance in time
            x(t) = x(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
            y(t) = y(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
            z(t) = z(t)+x_amp_dist*sin(2*pi*t*tdel*f_dist);
            phi(t) = phi(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);
            theta(t) = theta(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);
            %psi(t) = psi(t)+phi_amp_dist*sin(2*pi*t*tdel*f_dist);

            %% Addition of Noise
            x(t) = x(t)+x_amp_dist*randn;
            y(t) = y(t)+x_amp_dist*randn;
            z(t) = z(t)+x_amp_dist*randn;
            phi(t) = phi(t)+phi_amp_dist*randn;
            theta(t) = theta(t)+phi_amp_dist*randn;
            %psi(t) = psi(t)+phi_amp_dist*randn;
        end

        %% PD
        % [w1(t),w2(t),w3(t),w4(t)] = pd1(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,...
        %     KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv(t),z(t),...
        %     phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
        %     psi_vd,psid,psi_v(t),psi(t));

        %% PID
        % [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
        %     pid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),eI_y(t-1),...
        %     KzD,KzP,KzI,eI_z(t-1),KphiD,KphiP,KphiI, ...
        %     eI_phi(t-1),KthetaD,KthetaP,KthetaI,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
        %     max_w,xd(t),x(t),yd(t),y(t),zvd,zd,zv(t),z(t),...
        %     phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


        %% FSOM
        %             [w1(t),w2(t),w3(t),w4(t)] = fsom1(m,l,k,b,Ixx,Iyy,Izz,Kz,Kphi,...
        %                 Ktheta,Kpsi,max_w,zvd,zd,zv(t),z(t),...
        %                 phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
        %                 psi_vd,psid,psi_v(t),psi(t));

        % %             %% BSC
        %             [w1(t),w2(t),w3(t),w4(t)] = bsc1(m,l,k,b,Ixx,Iyy,Izz,Kz1,Kz2, Kphi1, ...
        %                 Kphi2,Ktheta1,Ktheta2,Kpsi1,Kpsi2,max_w,zvd,zd,zv(t),z(t),z(t-1),...
        %                 phi_vd,phid,phi_v(t),phi(t),phi(t-1),theta_vd,thetad,theta_v(t),theta(t),theta(t-1),...
        %                 psi_vd,psid,psi_v(t),psi(t),psi(t-1));

        %% SMC
            if NP == 0
                [w1(t),w2(t),w3(t),w4(t)] = smc1(m,l,k,b,Ixx,Iyy,Izz,Kz,Kz1,Kz2, Kphi,Kphi1, ...
                    Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,zvd,zd,zv(t),z(t),...
                    phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
                    psi_vd,psid,psi_v(t),psi(t));

% [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
%             pid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),eI_y(t-1),...
%             KzD,KzP,KzI,eI_z(t-1),KphiD,KphiP,KphiI, ...
%             eI_phi(t-1),KthetaD,KthetaP,KthetaI,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
%             max_w,xd(t),x(t),yd(t),y(t),zvd,zd,zv(t),z(t),...
%             phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


            elseif NP ==1
                % [w1(t),w2(t),w3(t),w4(t)] = smc1np(m,l,k,b,Ixx,Iyy,Izz,Kz,Kz1,Kz2, Kphi,Kphi1, ...
                %     Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,zvd,zd,zv(t),z(t),...
                %     phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
                %     psi_vd,psid,psi_v(t),psi(t));


                [w1(t),w2(t),w3(t),w4(t)] = pd1np(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,...
                    KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv(t),z(t),...
                    phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
                    psi_vd,psid,psi_v(t),psi(t));
            % % 
            %         [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
            % pid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),eI_y(t-1),...
            % KzD,KzP,KzI,eI_z(t-1),KphiD,KphiP,KphiI, ...
            % eI_phi(t-1),KthetaD,KthetaP,KthetaI,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
            % max_w,xd(t),x(t),yd(t),y(t),zvd,zd,zv(t),z(t),...
            % phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


              maxn = 4500;
              
    
            end
            
        
        if n > maxn

            n = 1;

            % 
            % if NP == 0
            % 
            %     % [w1obs(t-1),w2obs(t-1),w3obs(t-1),w4obs(t-1),w1eff(t-1),w2eff(t-1),w3eff(t-1),w4eff(t-1), NP] = ...
            %     %     w_obs(m,l,k,b,Ixx,Iyy,Izz,w1(t-1),w2(t-1),w3(t-1),w4(t-1),...
            %     %     phi(t-1),theta(t-1),za(t-1),pa(t-1),qa(t-1),ra(t-1));
            % 
            % 
            %     [x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),phi(t),theta(t),psi(t),...
            %         phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t),pa(t),qa(t),ra(t)] = ...
            %         quad_in(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,N1(t-1)*w1eff(t-1),N2(t-1)*w2eff(t-1),N3(t-1)*w3eff(t-1),N4(t-1)*w4eff(t-1),...
            %         x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
            %         zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));
            % 
            %     % [w1obs(t),w2obs(t),w3obs(t),w4obs(t),w1eff(t),w2eff(t),w3eff(t),w4eff(t), NP] = ...
            %     %     w_obs(m,l,k,b,Ixx,Iyy,Izz,w1(t),w2(t),w3(t),w4(t),...
            %     %     phi(t),theta(t),za(t),pa(t),qa(t),ra(t));
            % 
            % elseif NP ==1 
            % 
            %     [x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),phi(t),theta(t),psi(t),...
            %     phi_v(t),theta_v(t),psi_v(t),p(t),q(t),r(t),pa(t),qa(t),ra(t)] = ...
            %     quad_in_np(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,N1(t-1)*w1(t-1),N2(t-1)*w2(t-1),N3(t-1)*w3(t-1),N4(t-1)*w4(t-1),...
            %     x(t-1),y(t-1),z(t-1),xv(t-1),yv(t-1),...
            %     zv(t-1),phi(t-1),theta(t-1),psi(t-1),p(t-1),q(t-1),r(t-1));
            % end


            %% PD xy
            % [w1(t),w2(t),w3(t),w4(t)] = xypd1(m,l,k,b,Ixx,Iyy,Izz,KxD,KxP,KyD,KyP,KzD,KzP,KphiDx,KphiPx,KthetaDx,...
            %     KthetaPx,KpsiD,KpsiP,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
            %     phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

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

            if NP == 0
                [w1(t),w2(t),w3(t),w4(t)] = xysmc1(m,l,k,b,Ixx,Iyy,Izz,Kx,Kx1,Kx2,Ky,Ky1,Ky2, Kz,Kz1,Kz2, Kphi,Kphi1_xy, ...
                Kphi2_xy, Ktheta,Ktheta1_xy,Ktheta2_xy,Kpsi,Kpsi1,Kpsi2,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
                phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));
 

                % [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
                %                 xypid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),KxD,KxP,KxI,eI_y(t-1),KyD,KyP,KyI,...
                %                 KzD,KzP,KzI,eI_z(t-1),KphiDx,KphiPx,KphiIx, ...
                %                 eI_phi(t-1),KthetaDx,KthetaPx,KthetaIx,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
                %                 max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
                %                 phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

            elseif NP ==1 
                % [w1(t),w2(t),w3(t),w4(t)] = xysmc1np(m,l,k,b,Ixx,Iyy,Izz,Kx,Kx1,Kx2,Ky,Ky1,Ky2, Kz,Kz1,Kz2, Kphi,Kphi1_xy, ...
                % Kphi2_xy, Ktheta,Ktheta1_xy,Ktheta2_xy,Kpsi,Kpsi1,Kpsi2,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
                % phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

                [w1(t),w2(t),w3(t),w4(t)] = xypd1np(m,l,k,b,Ixx,Iyy,Izz,KxD,KxP,KyD,KyP,KzD,KzP,KphiDx,KphiPx,KthetaDx,...
                KthetaPx,KpsiD,KpsiP,max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
                phi_vd,phi_v(t),phi(t),theta_vd,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));

              % [w1(t),w2(t),w3(t),w4(t),eI_x(t),eI_y(t),eI_z(t),eI_phi(t),eI_theta(t),eI_psi(t)] = ...
              %                   xypid1(tdel,m,l,k,b,Ixx,Iyy,Izz,eI_x(t-1),KxD,KxP,KxI,eI_y(t-1),KyD,KyP,KyI,...
              %                   KzD,KzP,KzI,eI_z(t-1),KphiDx,KphiPx,KphiIx, ...
              %                   eI_phi(t-1),KthetaDx,KthetaPx,KthetaIx,eI_theta(t-1),KpsiD,KpsiP,KpsiI,eI_psi(t-1),...
              %                   max_w,xvd,xd(t),xv(t),x(t),yvd,yd(t),yv(t),y(t),zvd,zd,zv(t),z(t),...
              %                   phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),psi_vd,psid,psi_v(t),psi(t));


            elseif NP ==2 


                [w1(t),w2(t),w3(t),w4(t)] = pd1np(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,...
                    KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv(t),z(t),...
                    phi_vd,phid,phi_v(t),phi(t),theta_vd,thetad,theta_v(t),theta(t),...
                    psi_vd,psid,psi_v(t),psi(t));
            end



        end

        % T_data = table(t,...
        %     w1(t-1),w1obs(t-1),w1eff(t-1),...
        %     w2(t-1),w2obs(t-1),w2eff(t-1),...
        %     w3(t-1),w3obs(t-1),w3eff(t-1),...
        %     w4(t-1),w4obs(t-1),w4eff(t-1),...
        %     x(t),y(t),z(t),...
        %     phi(t),theta(t),psi(t),...
        %     NP);
        % 
        % 
        % % 
        % % T_data = table(t,x(t),y(t),z(t),xv(t),yv(t),zv(t),xa(t),ya(t),za(t),...
        % %     phi(t),theta(t),psi(t),p(t),q(t),r(t),pa(t),qa(t),ra(t),...
        % %     w1(t-1),w2(t-1),w3(t-1),w4(t-1),w1(t),w2(t),w3(t),w4(t),...
        % %     w1obs(t-1),w2obs(t-1),w3obs(t-1),w4obs(t-1),w1eff(t-1),w2eff(t-1),w3eff(t-1),w4eff(t-1),...
        % %     NP);
        % 
        % if (t == 2)
        %     writetable(T_data, 'myData1.csv','WriteMode','Append',...
        %         'WriteRowNames',true);
        % else
        %      writetable(T_data, 'myData1.csv','WriteMode','Append',...
        %      'WriteVariableNames',false,'WriteRowNames',false);
        % end
    end

    figure(addnoise);
    subplot(2,1,1);
    qpl = plot(tspan,w1(1:timepoints), tspan,w2(1:timepoints),tspan,w3(1:timepoints), tspan,w4(1:timepoints));
    qpl(1).LineWidth = 1.5;
    qpl(2).LineWidth = 1.5;
    qpl(3).LineWidth = 1.5;
    qpl(4).LineWidth = 1.5;
    ax = gca;
    ax.FontSize = 14;
    legend('w1obs','w2obs','w3obs','w4obs','Location','northwest');
    title('Observed Motor Speed',FontSize=22);
    ylabel('Motor Speed)');
    xlabel('Time (s)');
    axis([0 80 0 2000]);
    grid;
    
    % subplot(4,1,2);
    % spl =  plot(tspan,w1eff(1:timepoints), tspan,w2eff(1:timepoints),tspan,w3eff(1:timepoints), tspan,w4eff(1:timepoints));
    % spl(1).LineWidth = 1.5;
    % spl(2).LineWidth = 1.5;
    % spl(3).LineWidth = 1.5;
    % spl(4).LineWidth = 1.5;
    % ax = gca;
    % ax.FontSize = 14;
    % legend('w1','w2','w3','w4','Location','northwest');
    % title('Inputted Motor Speed for Faulty Motor',FontSize=22);
    % ylabel('Motor Speed');
    % xlabel('Time (s)');
    % axis([00 80 0 2001]);
    % grid;

    % subplot(4,1,3);
    % qpl = plot( tspan, w3(1:timepoints), tspan, w3eff(1:timepoints));
    % ax = gca;
    % ax.FontSize = 14;
    % legend('w1','w1','w3','w4','Location','northwest');
    % title('Motor Speed',FontSize=22);
    % ylabel('Motor Angular Velocity (Deg/s)');
    % xlabel('Time (s)');
    % axis([0 80 0 2000]);
    % grid;
    % 
    % subplot(4,1,4);
    % rpl = plot( tspan, w4(1:timepoints), tspan, w4eff(1:timepoints));
    % rpl(1).LineWidth = 1;
    % rpl(2).LineWidth = 1;
    % % rpl(3).LineWidth = 1;
    % ax = gca;
    % ax.FontSize = 14;
    % legend('w1eff','w1eff','w3eff','w4eff','Location','northwest');
    % title('Motor Speed',FontSize=22);
    % ylabel('Motor Angular Velocity (Deg/s)');
    % xlabel('Time (s)');
    % axis([00 80 -1000 1000]);
    % grid;
    % subplot(4,1,3);
    % %plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd,tspan , zd);
    % ppl = plot(tspan, x,':' ,tspan, y,'--', tspan, z,tspan, xd,tspan, yd);
    % ppl(1).LineWidth = 2;
    % ppl(2).LineWidth = 2;
    % ppl(3).LineWidth = 2;
    % ppl(4).LineWidth = 2;
    % ax = gca;
    % ax.FontSize = 14;
    % legend('x','y','z','desired xy','Location','northwest');
    % title('Translational Motion',FontSize=22);
    % ylabel('Movement (m)');
    % xlabel('Time (s)');
    % axis([0 80 -5 40]);
    % grid;


    subplot(2,1,2);
    rpl =plot(tspan, phi*180/pi,':',tspan, theta*180/pi,'--', tspan, psi*180/pi);
    rpl(1).LineWidth = 2;
    rpl(2).LineWidth = 2;
    rpl(3).LineWidth = 2;
    ax = gca;
    ax.FontSize = 14;
    legend('phi','theta','psi','Location','northwest');
    title('Angle Motion','FontSize',22);
    ylabel('Angle (Deg)');
    xlabel('Time (s)');
    axis([0 80 -80 80]);
    grid;

    
    % 
    % subplot(4,1,4);
    % spl = plot(tspan, phi_v*180/pi,':',tspan, theta_v*180/pi,'--', tspan, psi_v*180/pi);
    % spl(1).LineWidth = 1;
    % spl(2).LineWidth = 1;
    % spl(3).LineWidth = 1;
    % ax = gca;
    % ax.FontSize = 14;
    % legend('phi velocity','theta velocity','psi velocity','Location','northwest');
    % title('Angular Velocity','FontSize',22);
    % ylabel('Angular Velocity (Deg/s)');
    % xlabel('Time (s)');
    % axis([00 80 -1000 1000]);
    % grid;




end





function [x,y,z,xv,yv,zv,x_a,y_a,z_a,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r,p_a,q_a,r_a] = ...
    quad_in_np(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1,w2,w3,w4,xin,yin,zin,xvin,yvin,...
    zvin,phi_in,theta_in,psi_in,pin,qin,rin)

g = 9.81;

T = k*(w1*w1+w2*w2+w3*w3+w4*w4);
x_a = T/m *(cos(psi_in)*sin(theta_in))*cos(phi_in)+ ...
    sin(phi_in)*sin(psi_in)-1/m*Ax*xvin;
y_a = T/m *(sin(psi_in)*sin(theta_in)*cos(phi_in)- ...
    sin(phi_in)*cos(psi_in))-1/m*Ay*yvin;
z_a = -g+T/m*cos(phi_in)*cos(theta_in)-1/m*Az*zvin;

p_a = (Iyy-Izz)/Ixx*qin*rin- ...
    Ir/Ixx*qin*(-w1-w2+w3+w4) + ...
    k*l/Ixx*(w4*w4-w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(-w1-w2+w3+w4)+...
    k*l/Iyy*(w3*w3-w1*w1);

r_a = b/Izz*(-w1*w1-w2*w2+w3*w3+w4*w4);

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

function [x,y,z,xv,yv,zv,x_a,y_a,z_a,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r,p_a,q_a,r_a] = ...
    quad_in_imbalance_np(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1,w2,w3,w4,xin,yin,zin,xvin,yvin,...
    zvin,phi_in,theta_in,psi_in,pin,qin,rin)

g = 9.81;
imb1 = 1;
imb2 = 1;

T = k*(imb1*w1*imb1*w1+imb2*imb2*w2*w2+w3*w3+w4*w4);
x_a = T/m *(cos(psi_in)*sin(theta_in))*cos(phi_in)+ ...
    sin(phi_in)*sin(psi_in)-1/m*Ax*xvin;
y_a = T/m *(sin(psi_in)*sin(theta_in)*cos(phi_in)- ...
    sin(phi_in)*cos(psi_in))-1/m*Ay*yvin;
z_a = -g+T/m*cos(phi_in)*cos(theta_in)-1/m*Az*zvin;

p_a = (Iyy-Izz)/Ixx*qin*rin- ...
    Ir/Ixx*qin*(-imb1*w1-imb2*w2+w3+w4) + ...
    k*l/Ixx*(w4*w4-imb2*imb2*w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(-imb1*w1-imb2*w2+w3+w4)+...
    k*l/Iyy*(w3*w3-imb1*w1*imb1*w1);

r_a = b/Izz*(-imb1*w1*imb1*w1-imb2*imb2*w2*w2+w3*w3+w4*w4);

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

function [x,y,z,xv,yv,zv,x_a,y_a,z_a,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r,p_a,q_a,r_a] = ...
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
    Ir/Ixx*qin*(w1-w2+w3-w4) + ...
    k*l/Ixx*(w4*w4-w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(w1-w2+w3-w4)+...
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

function [x,y,z,xv,yv,zv,x_a,y_a,z_a,phi,theta,psi,phi_v,theta_v,psi_v,p,q,r,p_a,q_a,r_a] = ...
    quad_in_imbalance(tdel,m,l,k,b,Ixx,Iyy,Izz,Ir,Ax,Ay,Az,w1,w2,w3,w4,xin,yin,zin,xvin,yvin,...
    zvin,phi_in,theta_in,psi_in,pin,qin,rin)

g = 9.81;
imb1 = 1;
imb2 = 1;

T = k*(imb1*w1*imb1*w1+imb2*imb2*w2*w2+w3*w3+w4*w4);
x_a = T/m *(cos(psi_in)*sin(theta_in))*cos(phi_in)+ ...
    sin(phi_in)*sin(psi_in)-1/m*Ax*xvin;
y_a = T/m *(sin(psi_in)*sin(theta_in)*cos(phi_in)- ...
    sin(phi_in)*cos(psi_in))-1/m*Ay*yvin;
z_a = -g+T/m*cos(phi_in)*cos(theta_in)-1/m*Az*zvin;

p_a = (Iyy-Izz)/Ixx*qin*rin- ...
    Ir/Ixx*qin*(imb1*w1-imb2*w2+w3-w4) + ...
    k*l/Ixx*(w4*w4-imb2*imb2*w2*w2);
q_a = (Izz-Ixx)/Iyy*rin*pin-...
    -Ir/Iyy*pin*(imb1*w1-imb2*w2+w3-w4)+...
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

function [w1obs,w2obs,w3obs,w4obs,w1eff,w2eff,w3eff,w4eff] = ...
    w_obs(m,l,k,b,Ixx,Iyy,Izz,w1,w2,w3,w4,...
    phi,theta,z_a,p_a,q_a,r_a)


g = 9.81;
max_w = 2000;
max_margin = 200;

NP =0;

T_PD = (g + z_a)*m/cos(phi)/cos(theta);
torq_phi_PD = (p_a)*Ixx;
torq_theta_PD = (q_a)*Iyy;
torq_psi_PD = (r_a)*Izz;

w1obs = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w2obs = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
w3obs = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
w4obs = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);


K1 = w1/w1obs;
K2 = w2/w2obs;
K3 = w3/w3obs;
K4 = w4/w4obs;


w1eff = K1*w1obs;
w2eff = K2*w2obs;
w3eff = K3*w3obs;
w4eff = K4*w4obs;



if w1eff > (max_w - max_margin)
    w1eff = (max_w - max_margin);
elseif w1eff < 0
    w1eff =0;
end
if w2eff > (max_w - max_margin)
    w2eff = (max_w - max_margin);
elseif w2eff < 0
    w2eff =0;
end
if w3eff > (max_w - max_margin)
    w3eff = (max_w - max_margin);
elseif w3eff < 0
    w3eff =0;
end
if w4eff > (max_w - max_margin)
    w4eff = (max_w - max_margin);
elseif w4eff < 0
    w4eff =0;
end


if w1eff < w1-5
    w3eff = w3eff*w1eff/w1;
    w2eff = w2eff*(2-w1eff/w1);
    w4eff = w4eff*(2-w1eff/w1);
    NP =1

elseif w3eff < w3-5
    w1eff = w1eff*w3eff/w3;
    w2eff = w2eff*(2-w3eff/w3);
    w4eff = w4eff*(2-w3eff/w3);
    NP =1
end



if w2eff < w2-5
    w4eff = w4eff*w2eff/w2;
    w2eff = w2eff*(2-w1eff/w1);
    w4eff = w4eff*(2-w1eff/w1);
    NP =1
elseif w4eff < w4-5
    w2eff = w2eff*w4eff/w4;
    w1eff = w2eff*(2-w3eff/w3);
    w3eff = w4eff*(2-w3eff/w3);
    NP =1
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

function [w1,w2,w3,w4] = pd1np(m,l,k,b,Ixx,Iyy,Izz,KzD,KzP,KphiD,KphiP,KthetaD,KthetaP,KpsiD,KpsiP,max_w,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z))*m/cos(phi)/cos(theta);
torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi))*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta))*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi))*Izz;

% w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
% w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

factor = (2*b^2+k^2*l^2);

w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);


w1 = w1 * .1;
w2 = w2 * 1.9;
w3 = w3 * .1;
w4 = w4 *1.9;

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

function [w1,w2,w3,w4] = xypd1np(m,l,k,b,Ixx,Iyy,Izz,KxD,KxP,KyD,KyP,KzD,KzP,KphiD,KphiP,KthetaD,...
    KthetaP,KpsiD,KpsiP,max_w,xvd,xd,xv,x,yvd,yd,yv,y,zvd,zd,zv,z,...
    phivd,phiv,phi,thetavd,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

T_PD = (g + KzD*(zvd - zv) + KzP*(zd - z))*m/cos(phi)/cos(theta);

thetad = m/T_PD *(KxD*(xvd - xv) + KxP*(xd - x));
phid = - m/T_PD *(KyD*(yvd - yv) + KyP*(yd - y));

torq_phi_PD = (KphiD*(phivd - phiv)+KphiP*(phid-phi))*Ixx;
torq_theta_PD = (KthetaD*(thetavd-thetav)+KthetaP*(thetad-theta))*Iyy;
torq_psi_PD = (KpsiD*(psivd-psiv)+KpsiP*(psid-psi))*Izz;

% w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
% w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

factor = (2*b^2+k^2*l^2);

w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);


w1 = w1 * .5;
w2 = w2 * 1.5;
w3 = w3 * .5;
w4 = w4 *1.5;

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
factor = (2*b^2+k^2*l^2);

w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);

w1 = w1 * .1;
w2 = w2 * 1.9;
w3 = w3 * .1;
w4 = w4 *1.9;


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

factor = (2*b^2+k^2*l^2);

w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);


w1 = w1 * .0;
w2 = w2 * 2;
w3 = w3 * .0;
w4 = w4 *2

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

sz = -(zvd - zv)-Kz*(zd - z);
sphi = -(phivd - phiv)-Kphi*(phid - phi);
stheta = -(thetavd - thetav)-Ktheta*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi*(psid - psi);

T_PD = (g +(-Kz*Kz*(zd - z)-Kz1*sign(sz)-Kz2*sz))*m/cos(phi)/cos(theta);
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

function [w1,w2,w3,w4] = smc1np(m,l,k,b,Ixx,Iyy,Izz,Kz,Kz1,Kz2, Kphi,Kphi1, ...
    Kphi2, Ktheta,Ktheta1,Ktheta2,Kpsi,Kpsi1,Kpsi2,max_w,zvd,zd,zv,z,...
    phivd,phid,phiv,phi,thetavd,thetad,thetav,theta,psivd,psid,psiv,psi)

g = 9.81;

sz = -(zvd - zv)-Kz*(zd - z);
sphi = -(phivd - phiv)-Kphi*(phid - phi);
stheta = -(thetavd - thetav)-Ktheta*(thetad - theta);
spsi = -(psivd - psiv)-Kpsi*(psid - psi);

T_PD = (g +(-Kz*Kz*(zd - z)-Kz1*sign(sz)-Kz2*sz))*m/cos(phi)/cos(theta);
torq_phi_PD = (-Kphi*Kphi*(phid - phi)-Kphi1*sign(sphi)-Kphi2*sphi)*Ixx;
torq_theta_PD = (-Ktheta*Ktheta*(thetad - theta)-Ktheta1*sign(stheta)-Ktheta2*stheta)*Iyy;
torq_psi_PD = (-Kpsi*Kpsi*(psid - psi)-Kpsi1*sign(spsi)-Kpsi2*spsi)*Izz;


factor = (2*b^2+k^2*l^2);





% w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
% w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);


w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);



w1 = w1 * .5;
w2 = w2 * 1.5;
w3 = w3 * .5;
w4 = w4 *1.5;

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

function [w1,w2,w3,w4] = xysmc1np(m,l,k,b,Ixx,Iyy,Izz,Kx,Kx1,Kx2,Ky,Ky1,Ky2, Kz,Kz1,Kz2, Kphi,Kphi1, ...
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

% w1 = sqrt(T_PD/4/k-torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w2 = sqrt(T_PD/4/k-torq_phi_PD/2/k/l-torq_psi_PD/4/b);
% w3 = sqrt(T_PD/4/k+torq_theta_PD/2/k/l+torq_psi_PD/4/b);
% w4 = sqrt(T_PD/4/k+torq_phi_PD/2/k/l-torq_psi_PD/4/b);

factor = (2*b^2+k^2*l^2);

w1 = sqrt(T_PD/4/k +torq_phi_PD*b^2/k/l/factor - torq_theta_PD*(factor - b^2)/2/k/l/factor -torq_psi_PD*b/2/factor);
w2 = sqrt(T_PD/4/k -torq_phi_PD*(factor - b^2)/2/k/l/factor + torq_theta_PD*b^2/k/l/factor -torq_psi_PD*b/2/factor);
w3 = sqrt(T_PD/4/k -torq_phi_PD*b^2/k/l/factor + torq_theta_PD*(factor - b^2)/2/k/l/factor +torq_psi_PD*b/2/factor);
w4 = sqrt(T_PD/4/k +torq_phi_PD*(factor - b^2)/2/k/l/factor - torq_theta_PD*b^2/k/l/factor +torq_psi_PD*b/2/factor);


w1 = w1 * .5;
w2 = w2 * 1.5;
w3 = w3 * .5;
w4 = w4 *1.5;

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

