% Pixhawk
clc;
clear all;
close;

parameters

global Omg_1 Omg_2 Omg_3 Omg_4;

%%

t_final = 2;
t_interval = 0.001;

init_val = [0 0 0 0 0 0 0 0 0 0 0 0];

t_m = []; % TIME VECTOR

phi_m = []; theta_m = []; psi_m = []; z_im = [];

%% Loop

for i = 0:t_interval:t_final
    % Sensor data
    phi_ = init_val(1,1);
    theta_ = init_val(1,2);
    psi_ = init_val(1,3);

    x_i = init_val(1,4);
    y_i = init_val(1,5);
    z_i = init_val(1,6);

    u_ = init_val(1,7);
    v_ = init_val(1,8);
    w_ = init_val(1,9);

    p_ = init_val(1,10);
    q_ = init_val(1,11);
    r_ = init_val(1,12);


    % Data logging

    phi_m = [phi_m phi_]; theta_m = [theta_m theta_]; psi_m = [psi_m psi_]; 
    z_im = [z_im, z_i];

    % Control Inputs

    Omg_1 = sqrt(m_*g_/(4*b));
    Omg_2 = sqrt(m_*g_/(4*b));
    Omg_3 = sqrt(m_*g_/(4*b));
    Omg_4 = sqrt(m_*g_/(4*b));

    % Propagation using Ode45

    [t,Y_] = ode45(@Quad_plus_model_6DoF_noAct,[i,i+t_interval,i+2*t_interval],init_val);
    t_m = [t_m,t(2,:)];
    init_val = Y_(2,:);

end

%% Plots
figure;
subplot(2,2,1);
plot(t_m, phi_m*(180/pi));
xlabel("time (s)");
ylabel("\phi (deg)");

subplot(2,2,2);
plot(t_m, theta_m*(180/pi));
xlabel("time (s)");
ylabel("\theta (deg)");

subplot(2,2,3);
plot(t_m, psi_m*(180/pi));
xlabel("time (s)");
ylabel("\psi (ged)");

subplot(2,2,4);
plot(t_m, z_im);
xlabel("time (s)");
ylabel("z (meters)")
