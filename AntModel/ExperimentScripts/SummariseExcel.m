%Find the average goal stability for every experiment, and compare against
%test criterion
%For each subfolder, sum the values for each excel trial
clear all;
%Import the folder name to process
folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';

folderStruct = dir(folder_name);
resultsTable = table();
experimentIndex = 1;
%For all folders that are not hidden (experiments)
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

        experimentQuality = [experimentTable.Volume(:), experimentTable.Epsilon(:), experimentTable.COMOffset(:)];
        
        resultsTable.meanVolume(experimentIndex) = mean(experimentQuality(:,1));
        resultsTable.varVolume(experimentIndex) = var(experimentQuality(:,1));
        resultsTable.minmaxVolume(experimentIndex,:) = [min(experimentQuality(:,1)) , max(experimentQuality(:,1))];
        
        resultsTable.meanEpsilon(experimentIndex) = mean(experimentQuality(:,2));
        resultsTable.varEpsilon(experimentIndex) = var(experimentQuality(:,2));
        resultsTable.minmaxEpsilon(experimentIndex,:) = [min(experimentQuality(:,2)) , max(experimentQuality(:,2))];

        resultsTable.meanCOMOffset(experimentIndex) = mean(experimentQuality(:,3));
        resultsTable.varCOMOffset(experimentIndex) = var(experimentQuality(:,3));
        resultsTable.minmaxOffset(experimentIndex,:) = [min(experimentQuality(:,3)) , max(experimentQuality(:,3))];

        resultsTable.Properties.RowNames{experimentIndex} = folderStruct(i).name;
        experimentIndex = experimentIndex + 1;

    end


end

save([folder_name, '\summarisegoalexcel.mat'], 'resultsTable');
