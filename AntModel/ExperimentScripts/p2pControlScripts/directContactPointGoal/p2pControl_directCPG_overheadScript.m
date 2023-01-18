%% Overhead script to run all experiments for direct contact point goal generation with all point to point control methods
% Fixed variance
% 2-40 contacts
% Scripts contained
%     Grasp Mode            Point Control Mode
%     -----------           ---------------------
% 1.  IPD                   Random Search Space
% 2.  IPD                   Contact GMM Centred
% 3.  Align                 Random Search Space
% 4.  Align                 Contact GMM Centred
% 5.  IPD&Align             Random Search Space
% 6.  IPD&Align             Contact GMM Centred

%Warning: current folder must be the one that contains the overhead script

%1
run('.\IPD_RSS.m')

%2
run('.\IPD_CGMMC_fixvar.m')

%3
run('.\Align_RSS.m')

%4
run('.\Align_CGMMC_fixvar.m')

%5
run('.\IPDAlign_RSS.m')

%6
run('.\IPDAlign_CGMMC_fixvar.m')

%%6.1
%%run('.\IPDAlign_CGMMC_vardec.m')

%%6.2
%%run('.\IPDAlign_CGMMC_varinc.m')