%% Overhead script to run all experiments
% 2-40 contacts

% Scripts contained
%     Antenna Control Mode         Grasp Synth Style
%     -------------------         ------------------
% 1.    Joint Control              Direct Contact Point
% 2.    Joint Control              PCA
% 3.    Point to Point             Direct Contact Point
% 4.    Point to Point             PCA

%Warning: current folder must be the one that contains the overhead script

%1
% run('.\dice_jointControlScripts\directContactPointGoal\dice_jointControl_directCPGoal_overheadScript.m')
% run('.\plank_jointControlScripts\directContactPointGoal\plank_jointControl_directCPGoal_overheadScript.m')
run('.\wedge_jointControlScripts\directContactPointGoal\wedge_jointControl_directCPGoal_overheadScript.m')

%2
% run('.\dice_jointControlScripts\PCAGoal\dice_jointControl_PCAGoal_overheadScript.m')
% run('.\plank_jointControlScripts\PCAGoal\plank_jointControl_PCAGoal_overheadScript.m')
run('.\wedge_jointControlScripts\PCAGoal\wedge_jointControl_PCAGoal_overheadScript.m')

%3
% run('.\dice_p2pControlScripts\directContactPointGoal\dice_p2pControl_directCPG_overheadScript.m')
% run('.\plank_p2pControlScripts\directContactPointGoal\plank_p2pControl_directCPG_overheadScript.m')
run('.\wedge_p2pControlScripts\directContactPointGoal\wedge_p2pControl_directCPG_overheadScript.m')


%4
% run('.\dice_p2pControlScripts\PCAGoal\dice_p2pControl_PCAGoal_overheadScript.m')
% run('.\plank_p2pControlScripts\PCAGoal\plank_p2pControl_PCAGoal_overheadScript.m')
run('.\wedge_p2pControlScripts\PCAGoal\wedge_p2pControl_PCAGoal_overheadScript.m')

