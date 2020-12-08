close all;
clc;
clear;

%% Initial Condtions of the traj.
p_0 = [0,0,20];
v_0 = [0.25,0,-0.5];
a_0 = [0,0.5,0];

amp = 0.25; %0.25;
w = 0.2*pi; %0.0795; 
sample = 80; %200;
t_down = sample/2; %40
t_c = t_down;
z_0 = p_0(3);
i = 0;
dt = 0.2;
K = 20;

%soft constraints
v_u = [6,6,6];
v_d = [-6,-6,-1];
a_u = [3,3,3];
a_d = [-3,-3,-1];


%% Plot the reference traj.
for t = 0:dt:t_down
    
    i = i + 1;
    
    px(i) =  amp * t * cos(w * t);
    py(i) =  amp * t * sin(w * t);
    pz(i) = 20 - 0.5 * t;
    
    vx(i) = amp*cos(w*t)-amp*w*t.*sin(w*t);
    vy(i) = amp*sin(w*t)+amp*w*t.*cos(w*t);
    vz(i) = -0.5;
    
    ax(i) = -amp*w*sin(w*t)-amp*w*sin(w*t)-amp*w^2*t.*cos(w*t);
    ay(i) = amp*w*cos(w*t)+amp*w*cos(w*t)-amp*w^2*t.*sin(w*t);
    az(i) = 0;
    
end

t = 0:dt:t_down;

figure(1)

subplot(3,1,1)
plot(t,px)
hold on
plot(t,py)
hold on 
plot(t,pz)
legend('px','py','pz');
title('Position');

subplot(3,1,2)
plot(t,vx)
hold on
plot(t,vy)
hold on 
plot(t,vz)
legend('vx','vy','vz');
title('Velocity');


subplot(3,1,3)
plot(t,ax)
hold on
plot(t,ay)
hold on 
plot(t,az)
legend('ax','ay','az');
title('Acceleration');

figure(2)
plot3(px, py, pz);
title('QP based MPC tracking conical spiral')
hold on

%% MPC
j = 1;

log = [0 p_0 v_0 a_0];
plot3(log(j,2),log(j,3),log(j,4),'bo',...
            'MarkerSize',3,...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','b')
hold on

i= 0;
for t = dt:dt:t_down

    j = j + 1;
    t_f = t+K*dt-dt;
    p = zeros(K,3);
    v = zeros(K,3);
    a = zeros(K,3);
    
    for t_p = t:dt:t_f
        
        i = i + 1;
        
        p(i,1) = amp*t_p.*cos(w*t_p);
        p(i,2) = amp*t_p.*sin(w*t_p);
        p(i,3) = p_0(3)+ v_0(3)*t_p;
        
        v(i,1) = amp*cos(w*t_p)-amp*w*t_p.*sin(w*t_p);
        v(i,2) = amp*sin(w*t_p)+amp*w*t_p.*cos(w*t_p);
        v(i,3) = -0.5+0.5/(1+1/dt*t_down)*i;
        
        a(i,1) = -amp*w*sin(w*t_p)-amp*w*sin(w*t_p)-amp*w^2*t_p.*cos(w*t_p);
        a(i,2) = amp*w*cos(w*t_p)+amp*w*cos(w*t_p)-amp*w^2*t_p.*sin(w*t_p);
        a(i,3) = 0.5/(1+1/dt*t_down);
        
    end
    
    i = 0;
    
    for k =1:3
        p0 = p_0(k);
        v0 = v_0(k);
        a0 = a_0(k);
        vu = v_u(k);
        vd = v_d(k);
        au = a_u(k);
        ad = a_d(k);
        p_r = p(:,k);
        v_r = v(:,k);
        a_r = a(:,k);
        log_ = QP_based_MPC_Tracking(k, K, dt, p_r, v_r, a_r, p0, v0, a0,vu,vd,au,ad,t);
        p_0(k) = log_(2);
        v_0(k) = log_(3);
        a_0(k) = log_(4);
    end

    log = [log;t p_0 v_0 a_0];
    
    plot3(log(j,2),log(j,3),log(j,4),'bo',...
            'MarkerSize',3,...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','b');
    drawnow 
    %pause(0.03)
end

t = 0:dt:t_down;

figure(3)

subplot(3,1,1)
plot(t,log(:,2))
hold on
plot(t,log(:,3))
hold on 
plot(t,log(:,4))
legend('px','py','pz');
title('Position of tracking');

subplot(3,1,2)
plot(t,log(:,5))
hold on
plot(t,log(:,6))
hold on 
plot(t,log(:,7))
legend('vx','vy','vz');
title('Velocity of tracking');


subplot(3,1,3)
plot(t,log(:,8))
hold on
plot(t,log(:,9))
hold on 
plot(t,log(:,10))
legend('ax','ay','az');
title('Acceleration of tracking');
    
    