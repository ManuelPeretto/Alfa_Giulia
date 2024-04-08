function app = f_runmodel(app)
addpath 'C:\Users\manue\Documents\GitHub\Alfa_Giulia\Script Giulia\app\app_model_functions'
Solution = table;
Vehicle = f_defineVehicle(); % to define
Vehicle.choice_approx = app.Cos1CheckBox.Value;
operatingConditions = f_defineOperatingConditions(app); % to define (must give aSteer, vCar)
Tyre = f_defineTyres(app, Vehicle); % ok

[Tyre] = f_corneringStiffness(20,Vehicle,Tyre,app.VehicleModelDropDown.Value);

Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;

if app.TestDropDown.Value<3
 
dimension=size(operatingConditions.V_mat);  

for i_operatingConditions = 1:dimension(1)
    V_vec = operatingConditions.V_mat(i_operatingConditions,:);
    deltaf_vec = operatingConditions.deltaf_mat(i_operatingConditions,:);
    N = length(V_vec);
    Vehicle.iz=i_operatingConditions;
    switch app.VehicleModelDropDown.Value
        case {1 2}
            [Solution] = [Solution ; F_Singletrack_ss_app(V_vec,deltaf_vec,Vehicle,Tyre,N,app.VehicleModelDropDown.Value)]; % review outputs of function (all in one structure)

        case {3 4}
            [Solution] = [Solution ; F_Doubletrack_ss_app(V_vec,deltaf_vec,Vehicle,Tyre,N,app.VehicleModelDropDown.Value)]; % review outputs of function (all in one structure)
    end
    %Solution_mat(i_operatingConditions)=Solution;
end

else % if Constant radius
    for ii=1:numel(R)
        R_vec = zeros(1,N) + R(ii);  % vector steer angle fixed
        switch choice_model
            case {1 2}
                [Solution] = F_Singletrack_costant_radius_app(V_vec,R_vec,Vehicle,Tyre,N,choice_model);

            case {3 4}
                [Solution] = F_Doubletrack_costant_radius_app(V_vec,R_vec,Vehicle,Tyre,N,choice_model);
        end
    end
end
app.Solution = Solution;
end

function Tyre = f_defineTyres(app, Vehicle)

Tyre.Params_f = mfeval.readTIR(app.FrontTyresEditField.Value);
Tyre.Params_r = mfeval.readTIR(app.RearTyresEditField.Value);

% [Tyre] = F_Calcola_CS(20,Vehicle,Tyre,app.VehicleModelDropDown.Value);
%
% Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
% Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;
end
