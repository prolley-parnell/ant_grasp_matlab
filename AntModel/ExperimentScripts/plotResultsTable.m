%Plot the results from the resultsTable file

close all;
clear all;
%addpath('ExperimentOutput');

folder_name1 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\noRefinePlankObj2-40';
folder_name2 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\noRefine2-40';
folder_name3 = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFPlankObj2-40';
folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFDiceObj2-40';
file_name = 'summarisegoalqualityexcel.mat';

matStruct = load([folder_name, '\', file_name]);
resultsTable = matStruct.resultsTable;

for i=1:length(resultsTable.Properties.RowNames)
    name_split = split(resultsTable.Properties.RowNames(i),'_');
    numberOfContacts(i,:) = str2num(name_split{1});
end



meanT = [resultsTable.meanVolume, resultsTable.meanEpsilon, resultsTable.meanCOMOffset];
varT = [resultsTable.varVolume, resultsTable.varEpsilon, resultsTable.varCOMOffset];
minT = [resultsTable.minmaxVolume(:,1), resultsTable.minmaxEpsilon(:,1), resultsTable.minmaxOffset(:,1)];
maxT = [resultsTable.minmaxVolume(:,2), resultsTable.minmaxEpsilon(:,2), resultsTable.minmaxOffset(:,2)];

[NoC_ordered, i_ordered] = sort(numberOfContacts);

numberOfContacts = NoC_ordered;
meanT = meanT(i_ordered,:);
varT = varT(i_ordered,:);
minT = minT(i_ordered,:);
maxT = maxT(i_ordered,:);

variableNumber = 3;
names = {'Volume', 'Epsilon', 'COMOffset'};

%Remove outlier for no refine dice at 5 contacts


tiledlayout(variableNumber,1)
for j = 1:variableNumber
    nexttile
    hold on
    title(names{j})
    
    yyaxis left
    ylabel("Mean")
    %errorbar(numberOfContacts, meanT(:,j), minT(:,j), maxT(:,j))
    
    plot(numberOfContacts, meanT(:,j));

    

    yyaxis right
    ylabel("Variance")
    varplot = [numberOfContacts, varT(:,j)];
%     if j == 1
%         varplot(4,:) = [];
%     end
    plot(varplot(:,1),varplot(:,2));
    hold off
end
hold on
    xlabel("Number of Contacts");
    hold off


