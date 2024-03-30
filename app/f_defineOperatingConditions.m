function operatingConditions = f_defineOperatingConditions(app)
N = 200;
switch app.TestDropDown.Value
    case 1 % Ramp steer
        deltaf_vec=linspace(0,15,N); % vector steering angle fixed
        V_vec = [30:30:120];

        [operatingConditions.V_mat, operatingConditions.deltaf_mat] = meshgrid(V_vec, deltaf_vec); %% Vehicle
        if diff(size(operatingConditions.V_mat))<0
            operatingConditions.V_mat = operatingConditions.V_mat';
            operatingConditions.deltaf_mat = operatingConditions.deltaf_mat';
        end
    case 2 % Ramp speed
        Vmin = 1;     % [km/h]  Minimum speed
        Vmax = 120;   % [km/h]  Maximum speed

        V_vec=linspace(Vmin,Vmax,N)./3.6; % vector longitudinal speed Vx

        deltaf_vec = [5:5:20];   % [deg]  Input steer angle

        [operatingConditions.V_mat, operatingConditions.deltaf_mat] = meshgrid(V_vec, deltaf_vec); %% Vehicle

        if diff(size(operatingConditions.V_mat))<0
            operatingConditions.V_mat = operatingConditions.V_mat';
            operatingConditions.deltaf_mat = operatingConditions.deltaf_mat';
        end

    case 3 % Constant Radius
        Vmin = 1;     % [km/h]  Minimum speed
        Vmax = 120;   % [km/h]  Maximum speed

        V_vec=linspace(Vmin/3.6,Vmax/3.6,N); % vector longitudinal speed Vx

        R = [100,90,80,70];
end



end