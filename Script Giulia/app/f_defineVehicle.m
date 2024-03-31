function Vehicle = f_defineVehicle()
%% Parameters Input
Vehicle.m = 1449;                                   % total mass in Kg
Vehicle.J = 2129;                                   % rotational inertia of yaw motion
Vehicle.L = 2.82;                                   % Wheelbase
Vehicle.wd = 0.533687943;                           % Weight distribution
Vehicle.b = Vehicle.wd*Vehicle.L;                   % distance from gravity center to rear axle
Vehicle.a = Vehicle.L-Vehicle.b;                    % distance from gravity center to front axle
Vehicle.tau = 11.8;

Vehicle.Wf = 1.557;                                 % Wheel track front 
Vehicle.Wr = 1.625;                                 % Wheel track rear
Vehicle.h = 0.592;                                  % Gravity center height
Vehicle.dr = 0.086;                                 % height of rear roll center
Vehicle.df = 0.041;                                 % height of front roll center
Vehicle.dd = Vehicle.df+(Vehicle.dr-Vehicle.df)*Vehicle.a/Vehicle.L;

Vehicle.ks_f = 20.8e3;                              % N/m % front suspension stiffness
Vehicle.k_antiroll_f = 4.168e4;                     % front rollbar stiffness
Vehicle.k_roll_f = Vehicle.Wf^2 / 2 * (Vehicle.ks_f + 2*Vehicle.k_antiroll_f); % front roll stiffness

Vehicle.ks_r = 28.8e3;                              % N/m % rear suspension stiffness
Vehicle.k_antiroll_r = 1.1709e+04;                  % rear rollbar stiffness
Vehicle.k_roll_r = Vehicle.Wr^2 / 2 * (Vehicle.ks_r + 2 * Vehicle.k_antiroll_r); % rear roll stiffness

Vehicle.Kf_K = Vehicle.k_roll_f / (Vehicle.k_roll_f + Vehicle.k_roll_r);   % (Kf / Ktotale) Roll stiffness distribution to front axle

Vehicle.toe_f = 0;     % [deg]
Vehicle.toe_r = 0;     % [deg]
Vehicle.percentuale_Ack = 0;   % Ackermann 0 e 1


%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

Vehicle.camber_fl = 0;
Vehicle.camber_fr = 0;
Vehicle.camber_rl = 0;
Vehicle.camber_rr = 0;

end