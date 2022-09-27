%Plot the results from the resultsTable file

close all;
clear all;

folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\SimpleTactileTestScript';
file_name = 'summarisegoalexcel.mat';

matStruct = load([folder_name, '\', file_name]);
resultsTable = matStruct.resultsTable;

for i=1:length(resultsTable.Properties.RowNames)
    name_split = split(resultsTable.Properties.RowNames(i),'_');
    numberOfContacts(i,:) = str2num(name_split{1});
end

meanT = [resultsTable.meanVolume, resultsTable.meanEpsilon, -resultsTable.meanCOMOffset];
varT = [resultsTable.varVolume, resultsTable.varEpsilon, resultsTable.varCOMOffset]; 
variableNumber = 3;
names = {'Volume', 'Epsilon', 'COMOffset'};

tiledlayout(variableNumber,1)
for j = 1:variableNumber
    nexttile
    hold on
    title(names{j})
    xlabel = "Number of Contacts";
    
    yyaxis left
    ylabel("Mean")
    errorbar(numberOfContacts, meanT(:,j), varT(:,j)/2)

    yyaxis right
    ylabel("Variance")
    plot(numberOfContacts,varT(:,j))
    hold off
end

