clear all
close all
clc
% add source library is in the path
addpath(['.',filesep,'src',filesep]);
import casadi.*
%% Case setup
% method, noise, num_uncooperative, adaptive_safety
field_1 = 'method'; value_1 = repmat({'fxtm', 'ocbf', 'cbf'},1,3*2);
field_2 = 'num_uncooperative'; value_2 = repelem({0,1,2},3*2);
field_3 = 'noise'; value_3 = repmat(repelem({0,1},3),1,3);
% create permutation containing structure
options = struct(field_1, value_1,field_2, value_2,field_3, value_3);
%% Create file with header
file = strcat('.',filesep,'Results',filesep, 'results.csv'); 
if ~exist(file,"file")
    fid = fopen(file, 'w');
    fprintf(fid, strcat('TOD \t Method \t NumUncoop \t AdaptiveSafety',...
        '\t Noise \t Dx_ideal \t Dx_actual \t Dv_ideal \t Dv_actual',...
        '\t TotalD_ideal \t TotalD_actual',...
        '\t Energy_ideal \t Energy_actual',...
        '\t Acc_Diff_mean \t Acc_Diff_std \n'));
else
    fid = fopen(file, 'a+');
end
%% run simulation
for i=1:length(options)
    run_results(fid, "adaptive_safety", 1,"store_restuls",1,...
        "method",options(i).method,"noise",options(i).noise,...
        "num_uncooperative",options(i).num_uncooperative);
end
%% close files
fclose(fid);