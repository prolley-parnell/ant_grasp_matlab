% SCRIPT to Extract the Delta COC in experiments

%GPC_dcoc = GraphPlotClass();
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
% GPC_dcoc = GPC_dcoc.loadData('variables', {'contactsTable'}, [experimentFolder, '\dice']);

% Plot delta COM for one trial

%Find the COM from a single trial

%By proportionally updating the mean

% experimentDataStruct(1) = 'Align_CGMMC_fixvar' ( Location, Mean centred)
T = tiledlayout(4,4);
for E = 1:16
    nexttile
    hold on
    title(GPC_dcoc.experimentDataStruct(E).Title, 'Interpreter','none')
    reduceContTable = GPC_dcoc.experimentDataStruct(E).trialData(33).contactsTable(:,["Trial Number", "Time", "Contact Location"]);
    trialID = unique(reduceContTable.("Trial Number"));
    dataSummary = struct();
    for i = 1:length(trialID)
        trialData = reduceContTable(reduceContTable.("Trial Number") == trialID(i), ["Time", "Contact Location"]);
        COC = inf(height(trialData), 3);
        delta = inf(height(trialData), 1);
        for j = 1:height(trialData)
            COC(j,:) = mean(trialData{1:j,"Contact Location"}, 1);
            if j>=2
                delta(j,:) = vecnorm(COC(j-1,:)-COC(j,:), 2, 2);
            end
        end
        dataSummary(i).COC = COC;
        delta(isinf(delta)) = nan;
        dataSummary(i).delta = delta;
        xData = [2:length(COC)+1];
        xData(isnan(delta))=[];
        delta(isnan(delta))=[];
        dataSummary(i).xKnee = GPC_dcoc.findKnee(xData, delta);
    end
    
    histfit(cat(1, dataSummary(:).xKnee))
    hold off

end
A = dataSummary(22).delta;
[LT,ST,R] = trenddecomp(A)