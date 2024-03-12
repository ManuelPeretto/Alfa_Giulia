
%% select file tir

function [Tyre] = select_file_tir(pathstr)

folder = strcat(pathstr,'\file tir');
[file,path] = uigetfile('*.tir','Select a file .tir',folder);
if isequal(file,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
Tyre.Params_f = mfeval.readTIR(fullfile(path,file));
Tyre.Params_r = mfeval.readTIR(fullfile(path,file));
end