%% File to test the functions of the GraphPlotClass
% Created 23/01/2023

% GPC = GraphPlotClass();
% 
% %Add all Experiment Folders
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
% experimentDir = dir(experimentFolder);
% resultsFolderCell = {};
% for f = 1:length(experimentDir)
%     subfolderPath = [experimentFolder, '\', experimentDir(f).name];
%     if ~strcmp(experimentDir(f).name(1), '.') && isfolder(subfolderPath)
%         %"mat-files" is the subfolder name for .mat files in the runtime args
%         resultsFolderCell = [resultsFolderCell ; {subfolderPath}];
%     end
% end
% 
% %%
% GPC = GPC.addExperiment(resultsFolderCell)
% 
% save('GPC_Example.mat', 'GPC')
%%
experimentName = ["IPDAlign_RSS"];
measureName = ["Epsilon"];
[xData, yData] = GPC.extractMeasure(experimentName, measureName);



%% Visualise experiment
[p,tbl,stats] = anova1(yData{1})

%%
[medianVal, IQRVal] = GPC.findMedAndIQR(yData);
kneeX = GPC.findKnee(xData, medianVal)

%% Finding Cross Correlation
allExperimentName = {GPC.experimentDataStruct.Title};
allQualityName = ["Volume", "Epsilon", "normAlign", "COM Offset"];
[xData, yData] = GPC.extractMeasure(allExperimentName, allQualityName)

%%
%Compound all the measures

goalSample = reshape(yData{:}, [], 4);
[R, P] = corrcoef(goalSample,'Rows','complete')


%%
GPC.completePaperPlot


