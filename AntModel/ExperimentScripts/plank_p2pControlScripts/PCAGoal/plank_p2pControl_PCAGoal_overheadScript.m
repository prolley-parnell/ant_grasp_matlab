%% Overhead script to run all experiments for PCA goal generation with all p2p controlled methods
% Fixed variance
% 2-40 contacts
% Scripts contained
%     Grasp Mode            Joint Control Mode
%     -----------           ---------------------
% 1.  PCA                   Random Search Space
% 2.  PCA                   Contact GMM Var Static
% 3.  PCA                   Contact GMM Var Increase
% 4.  PCA                   Contact GMM Var Decrease



%Warning: current folder must be the one that contains the overhead script

%1
run('.\PCA_RSS.m')

%2
run('.\PCA_CGMMC_fixvar.m')


%3
%run('.\PCA_CGMMC_varinc.m')


%4
%run('.\PCA_CGMMC_vardec.m')

cd('C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts')
