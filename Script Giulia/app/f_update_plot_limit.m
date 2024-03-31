function f_update_plot_limit(app)

xlim(app.UIAxes,[app.XMinEditField.Value app.XMaxEditField.Value ]);
ylim(app.UIAxes,[app.YMinEditField.Value app.YMaxEditField.Value ]);

end