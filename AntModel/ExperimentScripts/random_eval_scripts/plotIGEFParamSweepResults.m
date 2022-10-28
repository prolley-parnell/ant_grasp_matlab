%PLOT IGEF Param Sweep results

close all;
clear all;


folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\IGEFParamSweep';
file_name = 'experimentStatTable.mat';

matStruct = load([folder_name, '\', file_name]);
varIdx = 4;
statTable = matStruct.IGEFSweepCellArray{varIdx};


meanT = [statTable.meanVolume, statTable.meanEpsilon, statTable.meanCOMOffset, statTable.meanNormAlign];
varT = [statTable.varVolume, statTable.varEpsilon, statTable.varCOMOffset, statTable.varNormAlign];
minT = [statTable.minmaxVolume(:,1), statTable.minmaxEpsilon(:,1), statTable.minmaxOffset(:,1), statTable.minmaxNormAlign(:,1)];
maxT = [statTable.minmaxVolume(:,2), statTable.minmaxEpsilon(:,2), statTable.minmaxOffset(:,2), statTable.minmaxNormAlign(:,2)];

[orderedParam, i_ordered] = sort(statTable.paramValue);

paramValueArray = orderedParam;
meanT = meanT(i_ordered,:);
varT = varT(i_ordered,:);
minT = minT(i_ordered,:);
maxT = maxT(i_ordered,:);

variableNumber = 4;
names = {'Volume', 'Epsilon', 'COMOffset', 'Surface Alignment'};


tiledlayout(variableNumber,1)
for j = 1:variableNumber
    nexttile
    hold on
    title(names{j})
    
    yyaxis left
    ylabel("Mean")
    %errorbar(numberOfContacts, meanT(:,j), minT(:,j), maxT(:,j))
    
    plot(paramValueArray, meanT(:,j));

    yyaxis right
    ylabel("Variance")
    varplot = [paramValueArray, varT(:,j)];

    plot(varplot(:,1),varplot(:,2));
    hold off
end
hold on
label_x = ['Values of parameter ', int2str(varIdx)];
xlabel(label_x);
hold off


