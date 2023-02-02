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
%run('.\jointControlScripts\directContactPointGoal\jointControl_directCPGoal_overheadScript.m')
%run('.\plank_jointControlScripts\directContactPointGoal\plank_jointControl_directCPGoal_overheadScript.m')

%2
%run('.\jointControlScripts\PCAGoal\jointControl_PCAGoal_overheadScript.m')
%run('.\plank_jointControlScripts\PCAGoal\plank_jointControl_PCAGoal_overheadScript.m')

%3
%run('.\p2pControlScripts\directContactPointGoal\p2pControl_directCPG_overheadScript.m')
run('.\plank_p2pControlScripts\directContactPointGoal\plank_p2pControl_directCPG_overheadScript.m')


%4
%run('.\p2pControlScripts\PCAGoal\p2pControl_PCAGoal_overheadScript.m')
%run('.\plank_p2pControlScripts\PCAGoal\plank_p2pControl_PCAGoal_overheadScript.m')

