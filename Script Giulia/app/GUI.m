classdef GUI < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        YMaxEditField              matlab.ui.control.NumericEditField
        YMaxEditFieldLabel         matlab.ui.control.Label
        YMinEditField              matlab.ui.control.NumericEditField
        YMinEditFieldLabel         matlab.ui.control.Label
        XMaxEditField              matlab.ui.control.NumericEditField
        XMaxEditFieldLabel         matlab.ui.control.Label
        XMinEditField              matlab.ui.control.NumericEditField
        XMinEditFieldLabel         matlab.ui.control.Label
        YDropDown                  matlab.ui.control.DropDown
        YDropDownLabel             matlab.ui.control.Label
        XDropDown                  matlab.ui.control.DropDown
        XDropDownLabel             matlab.ui.control.Label
        Panel                      matlab.ui.container.Panel
        Panel_2                    matlab.ui.container.Panel
        SelectButton_2             matlab.ui.control.Button
        SelectButton               matlab.ui.control.Button
        RunButton                  matlab.ui.control.Button
        RearTyresEditField         matlab.ui.control.EditField
        RearTyresEditFieldLabel    matlab.ui.control.Label
        FrontTyresEditField        matlab.ui.control.EditField
        FrontTyresEditFieldLabel   matlab.ui.control.Label
        Panel_3                    matlab.ui.container.Panel
        Cos1CheckBox               matlab.ui.control.CheckBox
        TestDropDown               matlab.ui.control.DropDown
        TestDropDownLabel          matlab.ui.control.Label
        VehicleModelDropDown       matlab.ui.control.DropDown
        VehicleModelDropDownLabel  matlab.ui.control.Label
        UIAxes                     matlab.ui.control.UIAxes
        ColorDropDown
        ColorDropDownLabel
        Solution
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: SelectButton_2
        function SelectButton_2Pushed(app, event)
            currentFile = mfilename('fullpath');
            [pathstr, ~, ~] = fileparts(currentFile);
            folder = strcat(pathstr,'\file tir');
            [file,path] = uigetfile('*.tir','Select a file .tir',folder);
            app.RearTyresEditField.Value = fullfile(path,file);
        end

        % Button pushed function: SelectButton
        function SelectButtonPushed(app, event)
            currentFile = mfilename('fullpath');
            [pathstr, ~, ~] = fileparts(currentFile);
            folder = strcat(pathstr,'\file tir');
            [file,path] = uigetfile('*.tir','Select a file .tir',folder);
            app.FrontTyresEditField.Value = fullfile(path,file);
        end

        % Callback function: not associated with a component
        function Model_select(app, event)

        end

        function callback_update_plot(app,event)
             app = f_update_plot(app);
             f_update_plot_limit(app);
        end    

        % Button pushed function: RunButton
        function RunButtonPushed(app, event)
%             app = main_callback(app); % TO DEFINE
              AA = 3;
              app = f_runmodel(app);
              app = f_runplot(app);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [1 1 1];
            app.UIFigure.Position = [100 100 652 512];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [2 1 435 267];

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.TitlePosition = 'centertop';
            app.Panel.FontWeight = 'bold';
            app.Panel.FontSize = 18;
            app.Panel.Position = [1 280 653 233];

            % Create Panel_3
            app.Panel_3 = uipanel(app.Panel);
            app.Panel_3.Position = [0 90 652 143];

            % Create VehicleModelDropDownLabel
            app.VehicleModelDropDownLabel = uilabel(app.Panel_3);
            app.VehicleModelDropDownLabel.HorizontalAlignment = 'center';
            app.VehicleModelDropDownLabel.FontSize = 18;
            app.VehicleModelDropDownLabel.FontWeight = 'bold';
            app.VehicleModelDropDownLabel.Position = [79 107 129 23];
            app.VehicleModelDropDownLabel.Text = 'Vehicle Model';

            % Create VehicleModelDropDown
            app.VehicleModelDropDown = uidropdown(app.Panel_3);
            app.VehicleModelDropDown.Items = {'Single Track Linear', 'Single Track Non Linear', 'Double Track Linear', 'Double Track Non Linear'};
            app.VehicleModelDropDown.FontSize = 18;
            app.VehicleModelDropDown.FontWeight = 'bold';
            app.VehicleModelDropDown.Position = [32 69 223 32];
            app.VehicleModelDropDown.Value = 'Single Track Linear';
            app.VehicleModelDropDown.ItemsData = 1:length(app.VehicleModelDropDown.Items) ;

            % Create TestDropDownLabel
            app.TestDropDownLabel = uilabel(app.Panel_3);
            app.TestDropDownLabel.HorizontalAlignment = 'right';
            app.TestDropDownLabel.FontSize = 18;
            app.TestDropDownLabel.FontWeight = 'bold';
            app.TestDropDownLabel.Position = [368 107 41 23];
            app.TestDropDownLabel.Text = 'Test';

            % Create TestDropDown
            app.TestDropDown = uidropdown(app.Panel_3);
            app.TestDropDown.Items = {'Ramp Steer', 'Ramp Speed', 'Constant Radius'};
            app.TestDropDown.FontSize = 18;
            app.TestDropDown.FontWeight = 'bold';
            app.TestDropDown.Position = [273 68 232 32];
            app.TestDropDown.Value = 'Ramp Steer';
            app.TestDropDown.ItemsData = 1:length(app.TestDropDown.Items) ;

            % Create Cos1CheckBox
            app.Cos1CheckBox = uicheckbox(app.Panel_3);
            app.Cos1CheckBox.Text = 'Cos(δ) ≠ 1';
            app.Cos1CheckBox.FontSize = 18;
            app.Cos1CheckBox.Position = [520 69 132 67];

            % Create Panel_2
            app.Panel_2 = uipanel(app.Panel);
            app.Panel_2.TitlePosition = 'centertop';
            app.Panel_2.BackgroundColor = [0.902 0.902 0.902];
            app.Panel_2.FontWeight = 'bold';
            app.Panel_2.FontSize = 18;
            app.Panel_2.Position = [1 -13 653 153];

            % Create FrontTyresEditFieldLabel
            app.FrontTyresEditFieldLabel = uilabel(app.Panel_2);
            app.FrontTyresEditFieldLabel.HorizontalAlignment = 'right';
            app.FrontTyresEditFieldLabel.Position = [19 113 65 22];
            app.FrontTyresEditFieldLabel.Text = 'Front Tyres';

            % Create FrontTyresEditField
            app.FrontTyresEditField = uieditfield(app.Panel_2, 'text');
            app.FrontTyresEditField.Position = [99 113 420 22];

            % Create RearTyresEditFieldLabel
            app.RearTyresEditFieldLabel = uilabel(app.Panel_2);
            app.RearTyresEditFieldLabel.HorizontalAlignment = 'right';
            app.RearTyresEditFieldLabel.Position = [21 67 63 22];
            app.RearTyresEditFieldLabel.Text = 'Rear Tyres';

            % Create RearTyresEditField
            app.RearTyresEditField = uieditfield(app.Panel_2, 'text');
            app.RearTyresEditField.Position = [99 67 420 22];

            % Create RunButton
            app.RunButton = uibutton(app.Panel_2, 'push');
            app.RunButton.ButtonPushedFcn = createCallbackFcn(app, @RunButtonPushed, true);
            app.RunButton.BackgroundColor = [0 1 0];
            app.RunButton.FontSize = 14;
            app.RunButton.Position = [273 16 107 39];
            app.RunButton.Text = 'Run';
            app.RunButton.Icon = fullfile(pathToMLAPP, 'Giulia.png');

            % Create SelectButton
            app.SelectButton = uibutton(app.Panel_2, 'push');
            app.SelectButton.ButtonPushedFcn = createCallbackFcn(app, @SelectButtonPushed, true);
            app.SelectButton.BackgroundColor = [1 1 0];
            app.SelectButton.Position = [538 112 100 23];
            app.SelectButton.Text = 'Select';

            % Create SelectButton_2
            app.SelectButton_2 = uibutton(app.Panel_2, 'push');
            app.SelectButton_2.ButtonPushedFcn = createCallbackFcn(app, @SelectButton_2Pushed, true);
            app.SelectButton_2.BackgroundColor = [1 1 0];
            app.SelectButton_2.Position = [538 66 100 23];
            app.SelectButton_2.Text = 'Select';

            % Create XDropDownLabel
            app.XDropDownLabel = uilabel(app.UIFigure);
            app.XDropDownLabel.HorizontalAlignment = 'right';
            app.XDropDownLabel.Position = [499 231 25 22];
            app.XDropDownLabel.Text = 'X ';

            % Create XDropDown
            app.XDropDown = uidropdown(app.UIFigure);
            app.XDropDown.Position = [539 231 100 22];
            app.XDropDown.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create YDropDownLabel
            app.YDropDownLabel = uilabel(app.UIFigure);
            app.YDropDownLabel.HorizontalAlignment = 'right';
            app.YDropDownLabel.Position = [499 195 25 22];
            app.YDropDownLabel.Text = 'Y';

            % Create YDropDown
            app.YDropDown = uidropdown(app.UIFigure);
            app.YDropDown.Position = [539 195 100 22];
            app.YDropDown.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create ColorDropDown
            app.ColorDropDown = uidropdown(app.UIFigure);
            app.ColorDropDown.Position = [539 159 100 22];
            app.ColorDropDown.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create ColorDropDownLabel
            app.ColorDropDownLabel = uilabel(app.UIFigure);
            app.ColorDropDownLabel.HorizontalAlignment = 'right';
            app.ColorDropDownLabel.Position = [499 159 25 22];
            app.ColorDropDownLabel.Text = 'Color';

            % Create XMinEditFieldLabel
            app.XMinEditFieldLabel = uilabel(app.UIFigure);
            app.XMinEditFieldLabel.HorizontalAlignment = 'right';
            app.XMinEditFieldLabel.Position = [460 77 35 22];
            app.XMinEditFieldLabel.Text = 'X Min';

            % Create XMinEditField
            app.XMinEditField = uieditfield(app.UIFigure, 'numeric');
            app.XMinEditField.Position = [510 77 39 22];
            app.XMinEditField.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create XMaxEditFieldLabel
            app.XMaxEditFieldLabel = uilabel(app.UIFigure);
            app.XMaxEditFieldLabel.HorizontalAlignment = 'right';
            app.XMaxEditFieldLabel.Position = [560 77 39 22];
            app.XMaxEditFieldLabel.Text = 'X Max';

            % Create XMaxEditField
            app.XMaxEditField = uieditfield(app.UIFigure, 'numeric');
            app.XMaxEditField.Position = [614 77 37 22];
            app.XMaxEditField.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create YMinEditFieldLabel
            app.YMinEditFieldLabel = uilabel(app.UIFigure);
            app.YMinEditFieldLabel.HorizontalAlignment = 'right';
            app.YMinEditFieldLabel.Position = [457 29 35 22];
            app.YMinEditFieldLabel.Text = 'Y Min';

            % Create YMinEditField
            app.YMinEditField = uieditfield(app.UIFigure, 'numeric');
            app.YMinEditField.Position = [513 29 40 22];
            app.YMinEditField.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Create YMaxEditFieldLabel
            app.YMaxEditFieldLabel = uilabel(app.UIFigure);
            app.YMaxEditFieldLabel.HorizontalAlignment = 'right';
            app.YMaxEditFieldLabel.Position = [558 29 39 22];
            app.YMaxEditFieldLabel.Text = 'Y Max';

            % Create YMaxEditField
            app.YMaxEditField = uieditfield(app.UIFigure, 'numeric');
            app.YMaxEditField.Position = [612 29 39 22];
            app.YMaxEditField.ValueChangedFcn = createCallbackFcn(app, @callback_update_plot, true);

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GUI

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end