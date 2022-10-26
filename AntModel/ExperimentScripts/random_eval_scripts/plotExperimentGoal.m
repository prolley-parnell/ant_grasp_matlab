%% Plot the results of the new experiment goals

%Plot the results from the resultsTable file

close all;
clear all;

folder_name = pwd;
file_name = 'graspQualityEval.mat';

matStruct = load([folder_name, '\', file_name]);
resultsTable = matStruct.resultsTable;

numberOfContacts = resultsTable.nContacts;
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

tiledlayout(variableNumber,1)
for j = 1:variableNumber
    nexttile
    hold on
    title(names{j})
    
    yyaxis left
    ylabel("Mean")
    errorbar(numberOfContacts, meanT(:,j), minT(:,j), maxT(:,j))

   
    yyaxis right
    ylabel("Variance")
    plot(numberOfContacts,varT(:,j))
    hold off
end
hold on
    xlabel("Number of Contacts");
    hold off


