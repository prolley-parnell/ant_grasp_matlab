%% File to test the functions of the GraphPlotClass
% Created 23/01/2023

% GPC = GraphPlotClass();
% 
% %Add all Experiment Folders
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
% GPC = GPC.loadData;
% save('GPC_Example.mat', 'GPC')
%% Writing Exclude Failed Grasps Code
experimentName = ["IPDAlign_RSS"];
measureName = ["Volume", "COM Offset","Epsilon", "normAlign"];
refineFlag = 0;
[~, skewVal] = GPC.experimentPDF(experimentName, measureName, refineFlag);

%% Refine the data (testing)
[~, yData] = GPC.extractMeasure(experimentName);
[refinedDataOut, successNumber, failureNumber] = GPC.excludeFailedGrasp(yData{1});
GPC.transformData(refinedDataOut)

%% Visualise experiment
% [p,tbl,stats] = anova1(yData{1})

%%
% [medianVal, IQRVal] = GPC.findMedAndIQR(yData);
% kneeX = GPC.findKnee(xData, medianVal)

%% Finding Cross Correlation
% allExperimentName = {GPC.experimentDataStruct.Title};
% allQualityName = ["Volume", "Epsilon", "normAlign", "COM Offset"];
% [xData, yData] = GPC.extractMeasure(allExperimentName, allQualityName)
% 
% %Compound all the measures
% 
% goalSample = reshape(yData{:}, [], 4);
% [R, P] = corrcoef(goalSample,'Rows','complete')


%%
%GPC.completePaperPlot


