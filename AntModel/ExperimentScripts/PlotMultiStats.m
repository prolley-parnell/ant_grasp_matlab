%Plot multiple results files on the same graph

%Plot the results from the resultsTable file

close all;
clear all;


folder_name{1} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\DistAndAlignDiceObj2-40';
folder_name{2} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\DistOnlyDiceObj2-40';
folder_name{3} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\AlignOnlyDiceObj2-40';

nFolder = length(folder_name);

file_name = 'experimentStatTable.mat';

variableNumber = 4;
names = {'Volume', 'Epsilon', 'COMOffset', 'Surface Alignment'};


meanTableCell = cell([1,nFolder]);
experimentLegend = cell([1,nFolder]);;

for m=1:nFolder

    matStruct = load([folder_name{m}, '\', file_name]);
    statTable = matStruct.statTable;

    experimentName = split(folder_name{m}, '\')
    experimentLegend{m} = experimentName(end,:);

    for i=1:length(statTable.Properties.RowNames)
        name_split = split(statTable.Properties.RowNames(i),'_');
        numberOfContacts(i,:) = str2num(name_split{1});
    end



    meanT = [statTable.meanVolume, statTable.meanEpsilon, statTable.meanCOMOffset, statTable.meanNormAlign];
    % varT = [statTable.varVolume, statTable.varEpsilon, statTable.varCOMOffset, statTable.varNormAlign];
    % minT = [statTable.minmaxVolume(:,1), statTable.minmaxEpsilon(:,1), statTable.minmaxOffset(:,1), statTable.minmaxNormAlign(:,1)];
    % maxT = [statTable.minmaxVolume(:,2), statTable.minmaxEpsilon(:,2), statTable.minmaxOffset(:,2), statTable.minmaxNormAlign(:,2)];

    [NoC_ordered, i_ordered] = sort(numberOfContacts);

    numberOfContacts = NoC_ordered;
    meanTableCell{m} = [numberOfContacts, meanT(i_ordered,:)];
    % varT = varT(i_ordered,:);
    % minT = minT(i_ordered,:);
    % maxT = maxT(i_ordered,:);



end




    tiledlayout(variableNumber,1)
    for i = 1:variableNumber
        for j = 1:nFolder
            nexttile
            hold on
            title(names{i})
            
            ylabel("Mean")
            meanExpData = meanTableCell{j}(:,2:end);
            expContact = meanTableCell{j}(:,1);
            plot(expContact, meanExpData(:,i), 'DisplayName', experimentLegend{j});

            % yyaxis right
            % ylabel("Variance")
            % varplot = [numberOfContacts, varT(:,j)];

            % plot(varplot(:,1),varplot(:,2));
            % hold off

        end
        hold off
    end
    hold on
        xlabel("Number of Contacts");
        hold off






