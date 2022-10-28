%Find the average goal stability for every experiment, and compare against
%test criterion
%For each subfolder, sum the values for each excel trial
clear all;
%Import the folder name to process
folder_name1 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\noRefinePlankObj2-40';
folder_name2 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\noRefine2-40';
folder_name3 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFPlankObj2-40';
folder_name4 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFDiceObj2-40';
folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\noRefineAlignDiceObj2-40';

folderStruct = dir(folder_name);
resultsTable = table();
resultArray = [];
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
        name_split = split(folderStruct(i).name,'_');
        nContactArray = repmat(str2double(name_split{1}),[length(experimentStruct),1]);
        resultArray = cat(1, resultArray, [nContactArray,experimentQuality]);


        experimentIndex = experimentIndex + 1;

    end


end

%%

save([folder_name, '\allQualities.mat'], 'resultArray');


%%
resultsTable = table();

[~, sortIndx] = sort(resultArray(:,1));
sortedResultArray = resultArray(sortIndx,:);

nContact = unique(sortedResultArray(:,1));

for k = 1:length(nContact)

    expSortIndx = find(sortedResultArray(:,1) == nContact(k));
    experimentQualityAll = sortedResultArray(expSortIndx,:);

    resultsTable.meanVolume(k) = mean(experimentQualityAll(:,2));
    resultsTable.varVolume(k) = var(experimentQualityAll(:,2));
    resultsTable.minmaxVolume(k,:) = [min(experimentQualityAll(:,2)) , max(experimentQualityAll(:,2))];

    resultsTable.meanEpsilon(k) = mean(experimentQualityAll(:,3));
    resultsTable.varEpsilon(k) = var(experimentQualityAll(:,3));
    resultsTable.minmaxEpsilon(k,:) = [min(experimentQualityAll(:,3)) , max(experimentQualityAll(:,3))];

    resultsTable.meanCOMOffset(k) = mean(experimentQualityAll(:,4));
    resultsTable.varCOMOffset(k) = var(experimentQualityAll(:,4));
    resultsTable.minmaxOffset(k,:) = [min(experimentQualityAll(:,4)) , max(experimentQualityAll(:,4))];

    resultsTable.Properties.RowNames{k} = int2str(nContact(k));
    
end

save([folder_name, '\summarisegoalqualityexcel.mat'], 'resultsTable');