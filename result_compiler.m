clear all
close all
clc
% add source library is in the path
addpath(['.',filesep,'src',filesep]);
import casadi.*
store_results = 0;
adaptive_safety = 0;
time_inc = 8;
plots = 0;
type = 'uncooperative'; % ablation, nominal, uncooperative

%% Case setup
switch type
    case 'ablation'
        % generate etable with ablation studytf
        % method, noise, num_uncooperative, adaptive_safety
        field_1 = 'method'; value_1 = repmat({'fxtm', 'ocbf', 'cbf'},1,2);
        field_2 = 'num_uncooperative'; value_2 = repelem({0,2},3);
        field_3 = 'noise'; value_3 = repelem({1},6);
    case 'nominal'
        % nominal case 
        field_1 = 'method'; value_1 = {'fxtm', 'ocbf', 'cbf'};
        field_2 = 'num_uncooperative'; value_2 = repelem({0},3);
        field_3 = 'noise'; value_3 = repelem({1},3);
    case 'uncooperative'
        % uncooperative case 
        field_1 = 'method'; value_1 = {'fxtm', 'ocbf', 'cbf'};
        field_2 = 'num_uncooperative'; value_2 = repelem({2},3);
        field_3 = 'noise'; value_3 = repelem({1},3);
    otherwise
        error('case nor defined')
end
% create permutation containing structure
options = struct(field_1, value_1,field_2, value_2,field_3, value_3);
%% Create file with header
file = strcat('.',filesep,'Results',filesep, 'results_adj3.csv'); 
if ~exist(file,"file") && store_results
    fid = fopen(file, 'w');
    fprintf(fid, strcat('TOD \t Method \t NumUncoop \t AdaptiveSafety',...
        '\t Noise \t Dx_ideal \t Dx_actual \t Dv_ideal \t Dv_actual',...
        '\t TotalD_ideal \t TotalD_actual',...
        '\t Energy_ideal \t Energy_actual',...
        '\t Acc_Diff_mean \t Acc_Diff_std \t t_o_l\n'));
elseif exist(file,"file") && store_results
    fid = fopen(file, 'a+');
else
    fid = 1;
end

%% run simulation
cav_env = [];
for i=1:length(options)
    [cav_env_i,tf, i_m]=...
        run_results(fid, plots,"adaptive_safety", adaptive_safety,...
        "store_restuls",store_results,"method",options(i).method,...
        "noise",options(i).noise,"added_time",time_inc,...
        "num_uncooperative",options(i).num_uncooperative);
    cav_env=cat(2,cav_env,cav_env_i);
end
try
    fclose(fid);
catch err
    warning(err.message)
end
%% Create plots 
if strcmp(type, 'nominal') || strcmp(type, 'uncooperative')
    location = strcat('.',filesep,'Results',filesep,type); 
    % status = mkdir(location);
    % Extract CAV C set
    cav_c_set = cav_env(end-1,:);
    Utils.plot_control_comparison(cav_c_set, location);
end