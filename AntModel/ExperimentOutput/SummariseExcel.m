%Find the average goal stability for every experiment, and compare against
%test criterion
%For each subfolder, sum the values for each excel trial
clear all;
%Import the folder name to process
folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\SimpleTactileTestScriptV2';

folderStruct = dir(folder_name);
resultsTable = table();
experimentIndex = 1;
%For all folders that are not hidden
for i = 1:length(folderStruct)
    if ~strcmp(folderStruct(i).name(1), '.') && isfolder([folder_name, '\', folderStruct(i).name])
        %"printout" is the subfolder name for xls files in the runtime args
        subfolder = ['\', folderStruct(i).name, '\printout'];
        subDir = [folder_name, subfolder];


        experimentStruct = dir([subDir, '\*.xls']);
        experimentTable = table();

        for j = 1:length(experimentStruct)
            fileName = [subDir, '\', experimentStruct(j).name];
            sheetNamesj = sheetnames(fileName);
            for k = 1:length(sheetNamesj)
                if strcmp(sheetNamesj(k), 'Sensed Goal')
                    trialTable = readtable(fileName, 'FileType','spreadsheet', 'Sheet', k);
                    experimentTable(j,:) = trialTable;

                end
            end


        end
        goalQuality(experimentIndex,:) = mean([experimentTable.Volume(:), experimentTable.Epsilon(:), experimentTable.COMOffset(:)], 1);
        resultsTable.meanVolume(experimentIndex) = goalQuality(experimentIndex,1);
        resultsTable.meanEpsilon(experimentIndex) = goalQuality(experimentIndex,2);
        resultsTable.meanCOMOffset(experimentIndex) = goalQuality(experimentIndex,3);
        resultsTable.Properties.RowNames{experimentIndex} = folderStruct(i).name;
        experimentIndex = experimentIndex + 1;

    end


end
resultsTable = sortrows(resultsTable,2);
save([folder_name, '\summarisegoalexcel.mat'], 'resultsTable');
