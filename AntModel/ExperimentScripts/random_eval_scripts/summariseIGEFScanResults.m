%Summarise IGEF results
%Goal is to find the mean value of different grasp quality measures for
%each value of IGEF parameter that is altered


%Find the average goal stability for every experiment, and compare against
%test criterion
%For each subfolder, sum the values for each excel trial
clear all;
%Import the folder name to process

folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\IGEFParamSweep';

folderStruct = dir(folder_name);

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

        experimentQuality = [experimentTable.Volume(:), experimentTable.Epsilon(:), experimentTable.COMOffset(:),  experimentTable.normAlign(:)];
        name_split = split(folderStruct(i).name,'_');
        name_double = str2double(name_split);
        IGEFVals = name_double([2,3,4,5])'*10^-2;
        repIGEFVals = repmat(IGEFVals,[j,1]);
        
        resultArray = cat(1, resultArray, [repIGEFVals,experimentQuality]);


        experimentIndex = experimentIndex + 1;

    end


end

%%

save([folder_name, '\allQualities.mat'], 'resultArray');


%% For all the different IGEF parameters, find the average goal quality for each value


nVar = 4;
IGEFSweepCellArray = cell([1,nVar]);
for n=1:nVar
    statTable = table();
    [~, sortIndx] = sort(resultArray(:,n));
    sortedResultArray = resultArray(sortIndx,:);

    paramValue = unique(sortedResultArray(:,n));

    for k = 1:length(paramValue)

        expSortIndx = find(sortedResultArray(:,n) == paramValue(k));
        experimentQualityAll = sortedResultArray(expSortIndx,:);

        statTable.paramValue(k) = paramValue(k);

        statTable.meanVolume(k) = mean(experimentQualityAll(:,5));
        statTable.varVolume(k) = var(experimentQualityAll(:,5));
        statTable.minmaxVolume(k,:) = [min(experimentQualityAll(:,5)) , max(experimentQualityAll(:,5))];

        statTable.meanEpsilon(k) = mean(experimentQualityAll(:,6));
        statTable.varEpsilon(k) = var(experimentQualityAll(:,6));
        statTable.minmaxEpsilon(k,:) = [min(experimentQualityAll(:,6)) , max(experimentQualityAll(:,6))];

        statTable.meanCOMOffset(k) = mean(experimentQualityAll(:,7));
        statTable.varCOMOffset(k) = var(experimentQualityAll(:,7));
        statTable.minmaxOffset(k,:) = [min(experimentQualityAll(:,7)) , max(experimentQualityAll(:,7))];

        statTable.meanNormAlign(k) = mean(experimentQualityAll(:,8));
        statTable.varNormAlign(k) = var(experimentQualityAll(:,8));
        statTable.minmaxNormAlign(k,:) = [min(experimentQualityAll(:,8)) , max(experimentQualityAll(:,8))];

        
    end
    IGEFSweepCellArray{n} = statTable;
end

save([folder_name, '\experimentStatTable.mat'], 'IGEFSweepCellArray');