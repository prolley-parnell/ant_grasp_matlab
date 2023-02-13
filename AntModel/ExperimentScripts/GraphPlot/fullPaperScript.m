%% File to run the appropriate comparisons and load code as necessary
% Created 06/02/2023

GPC_dice = GraphPlotClass();
GPC_plank = GraphPlotClass();
GPC_wedge = GraphPlotClass();
%Add all Experiment Folders
experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
GPC_dice = GPC_dice.loadData([experimentFolder, '\dice']);
GPC_plank = GPC_plank.loadData([experimentFolder, '\plank']);
GPC_wedge = GPC_wedge.loadData([experimentFolder, '\wedge']);


save('GPC_full.mat', 'GPC_dice', 'GPC_plank', "GPC_wedge")


%% Plot to show how the knee points are used to select percentage
experimentNameExample = "Align_RSS";
GPC_dice.plotKneeSelection(experimentNameExample)

%% Plot the full grid of experiments and their corresponding knee values
[~, ~, tilePlank] = GPC_plank.completePaperPlot()
subtitle(tilePlank, 'Plank Object')
[~, ~, tileDice] = GPC_dice.completePaperPlot()
subtitle(tileDice, 'Dice Object')
[~, ~, tileWedge] = GPC_wedge.completePaperPlot()
subtitle(tileWedge, 'Wedge Object')

%% Rank all plank experiments

[plank_resultTable] = GPC_plank.extractPlotAndCostData;
plank_maxKneeTable = GPC_plank.findMaxKnee(plank_resultTable);
plank05Table = GPC_plank.addPercentPassColumn(plank_maxKneeTable, 0.5);
[plankRank] = GPC_plank.addRank(plank05Table);
completePlankTable = GPC_plank.addSummaryRank(plankRank);


%% Rank all dice experiments
[dice_resultTable] = GPC_dice.extractPlotAndCostData;
dice_maxKneeTable = GPC_dice.findMaxKnee(dice_resultTable);
dice05Table = GPC_dice.addPercentPassColumn(dice_maxKneeTable, 0.5);
[diceRank] = GPC_dice.addRank(dice05Table);
completeDiceTable = GPC_dice.addSummaryRank(diceRank);

%% Rank all wedge experiments
[wedge_resultTable] = GPC_wedge.extractPlotAndCostData;
wedge_maxKneeTable = GPC_wedge.findMaxKnee(wedge_resultTable);
wedge05Table = GPC_wedge.addPercentPassColumn(wedge_maxKneeTable, 0.5);
[wedgeRank] = GPC_wedge.addRank(wedge05Table);
completeWedgeTable = GPC_wedge.addSummaryRank(wedgeRank);

%% Compare both dice and plank to see the difference in ranking
allMeasureSummaryTable = GPC_plank.summariseTableRank(completeDiceTable, completePlankTable, completeWedgeTable);
allMeasureSummaryTable.Properties.VariableNames{1} = 'Dice';
allMeasureSummaryTable.Properties.VariableNames{2} = 'Plank';
allMeasureSummaryTable.Properties.VariableNames{3} = 'Wedge';