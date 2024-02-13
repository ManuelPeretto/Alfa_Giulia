clc
close all
clear


[file,path] = uigetfile('*.tir');
if isequal(file,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,file)]);
end

tireParams = simscape.multibody.tirread(fullfile(path,file));

