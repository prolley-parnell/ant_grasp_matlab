%% File to test the functions of the GraphPlotClass
% Created 23/01/2023

%GPC = GraphPlotClass();
% 
% %Add all Experiment Folders
%experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
%GPC = GPC.loadData;
%save('GPC_Example.mat', 'GPC')



%% Writing Exclude Failed Grasps Code
experimentName = ["IPDAlign_RSS"];
measureName = ["Volume", "COM Offset","Epsilon", "Align"];
refineFlag = 0;
[~, skewVal] = GPC.experimentPDF(experimentName, measureName, refineFlag);

%% Refine the data (testing)
experimentName = ["IPDAlign_RSS"];
measureName = [GPC.all_quality_name, "Simulation Time", "Real World Time"];
[~, yData] = GPC.extractMeasure(experimentName, measureName);
[refinedDataOut, successNumber, failureNumber] = GPC.excludeFailedGrasp(yData{1});
GPC.transformMedian(refinedDataOut, measureName)

%% Visualise experiment
% [p,tbl,stats] = anova1(yData{1})

%%
experimentName = ["IPDAlign_RSS"];
measureName = ["COM Offset"];
[xData, yData] = GPC.extractMeasure(experimentName, measureName);
[medianVal, IQRVal] = GPC.findMedAndIQR(yData);
kneeX = GPC.findKnee(xData{1}, medianVal);

%% Finding Cross Correlation
% allExperimentName = {GPC.experimentDataStruct.Title};
% allQualityName = ["Volume", "Epsilon", "Align", "COM Offset"];
% [xData, yData] = GPC.extractMeasure(allExperimentName, allQualityName)
% 
% %Compound all the measures
% 
% goalSample = reshape(yData{:}, [], 4);
% [R, P] = corrcoef(goalSample,'Rows','complete')


%%
%[~, mapTable, resultCellArray] = GPC.completePaperPlot
[resultTable] = GPC.extractPlotAndCostData;
GPC.plotResults(resultTable);

%%
GPC.plotResults(resultTable)

%%
%[orderedResultTable] = GPC.orderGraspQualityPlateau(resultTable);

%%
[resultTable] = GPC.extractPlotAndCostData;
[orderedResultTable] = GPC.orderGraspQualityPlateau(resultTable);
GPC.plotOrderedResultTable(orderedResultTable)
%%
GPC.plotOrderedMedianTable(orderedResultTable)

%%
GPC.plotOrderedMedianQualityAndCost(orderedResultTable)
