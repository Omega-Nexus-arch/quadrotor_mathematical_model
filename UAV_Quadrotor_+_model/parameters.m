%all units are in SI

m_ = 0.7; % kg
l_ = 0.1; % meters
Ixx_ = 0.0012; % kgm^2
Iyy_ = 0.0012; % kgm^2
Izz_ = 0.0024; % kgm^2
Ixy_ = 0; % kgm^2
Iyx_ = 0; % kgm^2
Ixz_ = 0; % kgm^2
Izx_ = 0; % kgm^2
Iyz_ = 0; % kgm^2
Izy_ = 0; % kgm^2
R_ = 0.0635; % meters % Radius of Propeller
C_T = 0.032; % Thrust coefficient
C_Q = 0.05; % Torque coefficient
g_ = 9.81; % m/s^2

rho = 1.225; % kg/m^3 %air Density

b = 0.5 * pi * C_T * R_^4 * rho;
d = 0.5 * pi * C_Q * R_^5 * rho;