%% For the collected goals for a set of experiments, re-apply the quality
%measures for set RUNTIME force and friction

close all;
clear;

modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
scriptFolder = pwd;
cd(modelFolder)

folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';


addpath('Environment', 'Grasp', 'ExperimentOutput', 'ToolClasses');



%Import the folder name to process

folderStruct = dir(folder_name);
resultsStruct = struct('experiment_name', [], 'experimentGoalStruct', []);
experimentIndex = 1;
%For all folders that are not hidden (experiments)
for i = 1:length(folderStruct)
    if ~strcmp(folderStruct(i).name(1), '.') && isfolder([folder_name, '\', folderStruct(i).name])
        %"printout" is the subfolder name for xls files in the runtime args
        subfolder = ['\', folderStruct(i).name, '\printout'];
        subDir = [folder_name, subfolder];


        experimentStruct = dir([subDir, '\*.xls']);
        experimentGoal = goalStruct();

        for j = 1:length(experimentStruct)
            fileName = [subDir, '\', experimentStruct(j).name];
            sheetNamesj = sheetnames(fileName);
            for k = 1:length(sheetNamesj)
                if strcmp(sheetNamesj(k), 'Sensed Goal')
                    trialTable = readtable(fileName, 'FileType','spreadsheet', 'Sheet', k);
                    %experimentTable(j,:) = trialTable;

                    experimentGoal(j).midpoint = [trialTable.Midpoint_1, trialTable.Midpoint_2, trialTable.Midpoint_3];
                    experimentGoal(j).contact_point_array(1,:) = [trialTable.PointA_1, trialTable.PointA_2, trialTable.PointA_3];
                    experimentGoal(j).contact_point_array(2,:) = [trialTable.PointB_1, trialTable.PointB_2, trialTable.PointB_3];

                end
            end

        end
        resultsStruct(experimentIndex).experiment_name = folderStruct(i).name;
        resultsStruct(experimentIndex).experimentGoalStruct = experimentGoal;
        experimentIndex = experimentIndex + 1;

    end


end
cd(folder_name)
save('allExperimentGraspContacts.mat', 'resultsStruct')
