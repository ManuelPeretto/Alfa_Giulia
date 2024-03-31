function app = f_update_plot(app)


scatter(app.UIAxes,app.Solution.(app.XDropDown.Value),app.Solution.(app.YDropDown.Value),20,app.Solution.(app.ColorDropDown.Value),'filled');
grid(app.UIAxes,"on");
xlabel(app.UIAxes,app.XDropDown.Value);
ylabel(app.UIAxes,app.YDropDown.Value);
colormap(app.UIAxes,'turbo');
cb = colorbar(app.UIAxes);
ylabel(cb,app.ColorDropDown.Value);

end
