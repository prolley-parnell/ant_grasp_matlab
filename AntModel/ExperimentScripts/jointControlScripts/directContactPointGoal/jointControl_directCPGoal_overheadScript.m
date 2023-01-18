%% Overhead script to run all experiments for direct contact point goal generation with all joint controlled methods
% Fixed variance
% 2-40 contacts
% Scripts contained
%     Grasp Mode            Joint Control Mode
%     -----------           ---------------------
% 1.  IPD                   Random Reach and Pull
% 2.  IPD                   Contact Mean Centred
% 3.  Align                 Random Reach and Pull
% 4.  Align                 Contact Mean Centred
% 5.  IPD&Align             Random Reach and Pull
% 6.  IPD&Align             Contact Mean Centred

%Warning: current folder must be the one that contains the overhead script

%1
run('.\IPD_RRaP.m')

%3
run('.\Align_RRaP.m')

%5
run('.\IPDAlign_RRaP.m')
%5.5

run('\IPDAlign_CMC_fixvar.m')
%6
run('.\IPDAlign_CMC_vardec.m')

%6.5
run('.\IPDAlign_CMC_varinc.m')