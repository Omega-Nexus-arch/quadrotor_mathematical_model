% Contains the Newton-Euler 6Dof State Equations

function Xdot = Quad_plus_model_6DoF_noAct(t,x)

    parameters
    global Omg_1 Omg_2 Omg_3 Omg_4;

    phi_ = x(1);     % Euler Angles
    theta_ = x(2);
    psi_ = x(3);
    x_i = x(4);      % Inertial Cordinates
    y_i = x(5);
    z_i = x(6);
    u_ = x(7);       % Velocity Along body frame
    v_ = x(8);
    w_ = x(9);
    p_ = x(10);      % Angular velocity along body frame
    q_ = x(11);
    r_ = x(12);

    % ================= FORCES =================
    
    Fbx = 0;
    Fby = 0;
    Fbz = -(b*Omg_1^2 + b*Omg_2^2 + b*Omg_3^2 + b*Omg_4^2) + m_*g_;
    
    % ================= TORQUES =================
    
    tau_x = l_*b*(Omg_4^2 - Omg_2^2);
    tau_y = l_*b*(Omg_1^2 - Omg_3^2);
    tau_z = d*(-Omg_1^2 + Omg_2^2 - Omg_3^2 + Omg_4^2);
    
    % ================= ROTATION MATRIX =================
    
    R = [cos(theta_)*cos(psi_) ...
         sin(phi_)*sin(theta_)*cos(psi_) - cos(phi_)*sin(psi_) ...
         cos(phi_)*sin(theta_)*cos(psi_) + sin(phi_)*sin(psi_);
    
         cos(theta_)*sin(psi_) ...
         sin(phi_)*sin(theta_)*sin(psi_) + cos(phi_)*cos(psi_) ...
         cos(phi_)*sin(theta_)*sin(psi_) - sin(phi_)*cos(psi_);
    
         -sin(theta_) ...
         sin(phi_)*cos(theta_) ...
         cos(phi_)*cos(theta_)];
    
    % ================= POSITION KINEMATICS =================
    
    pos_dot = R*[u_; v_; w_];
    
    x_dot = pos_dot(1);
    y_dot = pos_dot(2);
    z_dot = pos_dot(3);
    
    % ================= EULER RATE EQUATIONS =================
    
    phi_dot   = p_ + q_*sin(phi_)*tan(theta_) + r_*cos(phi_)*tan(theta_);
    theta_dot = q_*cos(phi_) - r_*sin(phi_);
    psi_dot   = q_*sin(phi_)/cos(theta_) + r_*cos(phi_)/cos(theta_);
    
    % ================= TRANSLATIONAL DYNAMICS =================
    
    u_dot = r_*v_ - q_*w_ + Fbx/m_;
    v_dot = p_*w_ - r_*u_ + Fby/m_;
    w_dot = q_*u_ - p_*v_ + Fbz/m_;
    
    % ================= ROTATIONAL DYNAMICS =================
    
    p_dot = (tau_x + (Iyy_-Izz_)*q_*r_)/Ixx_;
    q_dot = (tau_y + (Izz_-Ixx_)*p_*r_)/Iyy_;
    r_dot = (tau_z + (Ixx_-Iyy_)*p_*q_)/Izz_;
    
    % ================= STATE DERIVATIVE =================
    
    Xdot = [
    phi_dot
    theta_dot
    psi_dot
    x_dot
    y_dot
    z_dot
    u_dot
    v_dot
    w_dot
    p_dot
    q_dot
    r_dot
    ];
    
    end