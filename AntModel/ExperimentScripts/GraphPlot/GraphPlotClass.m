%% Class to process experiment data and re-run graphs on command
% Created 23/01/23
classdef GraphPlotClass

    properties
        experimentPath = {};
        experimentDataStruct
        all_quality_name
        all_cost_name

        median_colour
        percent_colour
        cost_colour_rwt
        cost_colour_st


    end

    methods
        function obj = GraphPlotClass(~)
            %GraphPlotClass Constructor
            obj.experimentDataStruct = struct('Title', {}, ...
                'nContact', [], ...
                'trialData', struct('contactNumber', [], 'senseGoalTable', table.empty, 'costSummaryTable', table.empty, 'nTrial', []));

            obj.all_quality_name = ["Epsilon", "Volume", "COM Offset", "Align", "Within Reach"];
            obj.all_cost_name = ["Joint Change", "Total Contact Memory Bytes", "limbControlCalculationTime", "goalCalcTime", "Simulation Time", "Real World Time"];

            obj.median_colour = [0.9 0.3 0];
            obj.percent_colour = [0.9 0.75 0];
            obj.cost_colour_rwt = [0.15 0 0.9];
            obj.cost_colour_st = [0.5 0.4 1];
        end



        %% Functions to load MAT files

        function obj = loadData(obj, varargin)
            % Used to add all folders in the provided path, or the assumed
            % path to the class instance. WARNING: Does not overwrite
            % existing experiments but adds to the end. May result in
            % repeated data
            if isempty(varargin)
                folderPath = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
            else
                folderPath = varargin{:};
            end
            experimentDir = dir(folderPath);
            resultsFolderCell = {};
            for f = 1:length(experimentDir)
                subfolderPath = [folderPath, '\', experimentDir(f).name];
                if ~strcmp(experimentDir(f).name(1), '.') && isfolder(subfolderPath)
                    %"mat-files" is the subfolder name for .mat files in the runtime args
                    resultsFolderCell = [resultsFolderCell ; {subfolderPath}];
                end
            end

            %%
            obj = obj.addExperiment(resultsFolderCell);
            obj = obj.renameTableColumns;
        end

        function obj = addExperiment(obj, folderPathCell)
            nFolder = length(folderPathCell);
            for n = 1:nFolder
                %Add the path to the file to the class to record which
                %files have been processed
                obj.experimentPath{end+1} = folderPathCell{n};



                folderStruct = dir(folderPathCell{n});

                %For all folders that are not hidden (experiments)
                for i = 1:length(folderStruct)
                    if ~strcmp(folderStruct(i).name(1), '.') && isfolder([folderPathCell{n}, '\', folderStruct(i).name])
                        %"mat-files" is the subfolder name for .mat files in the runtime args
                        subfolder = ['\', folderStruct(i).name, '\mat-files'];

                        MATSubDir = [folderPathCell{n}, subfolder];

                        obj = obj.addMATDir(MATSubDir);

                    end
                end
            end
        end

        function obj = addMATDir(obj, MATFolderPath)

            MATDirStruct = dir([MATFolderPath, '\*.mat']);

            for i = 1:length(MATDirStruct)
                fileName = [MATFolderPath, '\', MATDirStruct(i).name];

                obj = obj.addTrial(fileName);


            end

        end

        function obj = addTrial(obj, MATFile)
            load(MATFile, 'senseGoalTable', 'costSummaryTable')
            [experimentTitle, experimentContact, trialNumber] = obj.addressFromPath(MATFile);

            existingExperimentFlag = ismember({obj.experimentDataStruct(:).Title}, experimentTitle);
            if any(existingExperimentFlag)
                entryIndex = find(existingExperimentFlag);
            else
                entryIndex = length(existingExperimentFlag) + 1;
                obj.experimentDataStruct(entryIndex).Title = experimentTitle;
            end



            existingContactNumberFlag = ismember(obj.experimentDataStruct(entryIndex).nContact(:), experimentContact);
            if any(existingContactNumberFlag)
                contactIndex = find(existingContactNumberFlag);
            else
                contactIndex = length(existingContactNumberFlag)+1;
                obj.experimentDataStruct(entryIndex).nContact(contactIndex) = experimentContact;
                obj.experimentDataStruct(entryIndex).trialData(contactIndex).contactNumber = experimentContact;
                obj.experimentDataStruct(entryIndex).trialData(contactIndex).senseGoalTable = table.empty;
                obj.experimentDataStruct(entryIndex).trialData(contactIndex).costSummaryTable = table.empty;
            end



            existingGoalData = obj.experimentDataStruct(entryIndex).trialData(contactIndex).senseGoalTable;
            existingCostData = obj.experimentDataStruct(entryIndex).trialData(contactIndex).costSummaryTable;
            newGoalData = [existingGoalData; senseGoalTable];
            newCostData = [existingCostData; costSummaryTable];
            obj.experimentDataStruct(entryIndex).trialData(contactIndex).senseGoalTable = newGoalData;
            obj.experimentDataStruct(entryIndex).trialData(contactIndex).costSummaryTable = newCostData;
            obj.experimentDataStruct(entryIndex).trialData(contactIndex).nTrial = size(newGoalData,1);
        end



        function obj = renameTableColumns(obj)
            %RENAMETABLECOLUMNS - Ensure the experiment data has
            %appropriately labled columns for plotting
            nExperiment = length(obj.experimentDataStruct);
            for a = 1:nExperiment
                nRun = length(obj.experimentDataStruct(a).trialData);
                for b = 1:nRun
                    obj.experimentDataStruct(a).trialData(b).senseGoalTable.Properties.VariableNames{11} = 'Align';
                    obj.experimentDataStruct(a).trialData(b).senseGoalTable.Properties.VariableNames{12} = 'Within Reach';

                end

            end
        end


        function [experimentTitle, nContacts, trialNumber] = addressFromPath(~, MATPath)
            folderCell = split(MATPath, '\');

            matName = split(folderCell{end}, {'_', '.'});
            trialNumber = str2num(matName{2});

            contactsFileName = split(folderCell{end-2}, {'_', '.'});
            nContacts = str2num(contactsFileName{1});

            experimentTitle = folderCell{end-3};

        end

        %%
        function [xData, yData] = extractMeasure(obj, experimentName, varargin)
            %% For a given experiment across all contact values, return the raw cost or quality measures
            %yData is a cell array which is 1xnExperiment, each cell is
            %nTrial x nContact x nMeasure
            if isempty(varargin)
                %If measure name is not specified, return all quality
                %measures
                measureName = [obj.all_quality_name,"Simulation Time", "Real World Time"];
            else
                measureName = [varargin{:}];
            end

            
            %Extract the relevant values
            experimentNameFlag = contains({obj.experimentDataStruct(:).Title}, experimentName, 'IgnoreCase', true);

            experimentIdx = find(experimentNameFlag);
            if length(experimentIdx) > length(experimentName)
                experimentIdx = [];
                for n = 1:length(experimentName)
                    experimentNameFlag = strcmp({obj.experimentDataStruct(:).Title}, experimentName(n));
                    if any(experimentNameFlag)
                        experimentIdx(n) = find(experimentNameFlag);
                    else
                        warning("experiment name not found")
                    end
                end
            end
            nExperiment = length(experimentIdx);
            xData = cell(nExperiment, 1);
            yData = cell(nExperiment, 1);
            for i = 1:nExperiment
                nRun = length(obj.experimentDataStruct(experimentIdx(i)).nContact);
                xData_contact = nan(1,nRun);
                nTrial = max([obj.experimentDataStruct(experimentIdx(i)).trialData(:).nTrial]);


                goalQualityNameFlag = ismember(measureName, obj.all_quality_name);
                costNameFlag = ismember(measureName, obj.all_cost_name(2:end));

                nMeasure = sum([goalQualityNameFlag, costNameFlag]);
                yData_measureValue = nan(nTrial, nRun, nMeasure);


                for r = 1:nRun
                    xData_contact(r) = obj.experimentDataStruct(experimentIdx(i)).trialData(r).contactNumber;

                    %Assign data to the output measure depending on
                    %location of measure name


                    yData_measureValue(:,r,goalQualityNameFlag==1) = table2array(obj.experimentDataStruct(experimentIdx(i)).trialData(r).senseGoalTable(:,measureName(goalQualityNameFlag)));

                    yData_measureValue(:,r,costNameFlag==1) = table2array(obj.experimentDataStruct(experimentIdx(i)).trialData(r).costSummaryTable(:,measureName(costNameFlag)));

                end

                [xData{i}, i_ordered] = sort(xData_contact);
                yData{i} = yData_measureValue(:,i_ordered,:);

            end

        end

        function [medianVal, IQRVal] = findMedAndIQR(~, dataIn)
            %FINDMEDANDIQR DataIn is not transformed and is in a matrix n x m
            % where n is the number of data points and m is the number of
            % sample sets
            n = size(dataIn, 1);
            m = size(dataIn, 2);
            medianVal = median(dataIn{:}, 1);
            IQRVal = iqr(dataIn{:}, 1);
        end

        function [xValKnee] = findKnee(~, xData, medianSetIn)
            %FINDKNEE Apply the Knee algorithm to identify the number of
            %contacts at which the rate of improvement drops off
            %1: Smooth out the data
            x_s = xData;
            %Requires Curve Fitting Toolbox
            y_s = smooth(medianSetIn)';

            %plot(x_s, y_s);
            %2: Normalise to a unit square
            x_sn = rescale(x_s);
            y_sn = rescale(y_s);

            %3: Calculating the difference curve changes
            x_d = x_sn;

            %Select y_d based on differential
            % Find the slope direction
            try
                slope_dir = sign(y_sn(end) - y_sn(1));

            catch
                e = 1;
            end
            if slope_dir < 1
                y_d = 1 - (x_sn + y_sn);
            else
                y_d = y_sn - x_sn;
            end

            %plot(x_d, y_d)




            %4: Calculating the local maxima of the difference curve
            y_lm_flag = islocalmax(y_d);


            y_lm = y_d(y_lm_flag==1);
            x_lm = x_d(y_lm_flag==1);

            %plot(x_d,y_d,x_lm,y_lm,'r*')

            %5: Calculating threshold for each local maximum in the difference curve
            %S is sensitivity
            S = 1;
            x_sn_p1 = x_sn;
            n = length(x_sn);
            x_sn_p1(1) = [];
            T_lm_x = y_lm - S*(sum(x_sn_p1 - x_sn(1:n-1)) / (n-1));

            %6: Each difference value is compared with threshold
            lm_i = find(y_lm_flag);
            % Add another index to check the last segment
            lm_i(end+1) = length(x_sn);
            nSect = length(y_lm);
            knee_flag = zeros(1, nSect);
            for i = 1:nSect
                sect = y_d(lm_i(i):lm_i(i+1)-1);
                knee_flag(i) = any(sect < T_lm_x(i));

            end
            x = x_lm(knee_flag==1);
            try
                knee_x_sn = x(1);
            catch
                e =1;
            end
            %hold on
            %plot(x_lm, T_lm_x)

            % Convert back to the actual range
            kneeVal = min(xData) + (knee_x_sn * (max(xData) - min(xData)));

            % Round to the nearest input xData
            [~, idx] = min(abs(xData - kneeVal));
            xValKnee = xData(idx);

            %plot(xData, medianSetIn, xValKnee, medianSetIn(idx), 'r*')


        end

        %% Data Refinement and Transformations
        function [refinedDataOut, successNumber, failureNumber] = excludeFailedGrasp(obj, experimentDataIn)
            %EXCLUDEFAILEDGRASP For all trials, exclude any grasps that are
            %classed as failing based on any of the measures
            if iscell(experimentDataIn)
                experimentDataIn = experimentDataIn{1};
            end


            %WARNING: Expects the order layer to match that found in
            %obj.all_measure_name
            %Define the masks for different measures
            alignLayerID = find(strcmp(obj.all_quality_name, "Align"));
            alignFailMask = find(experimentDataIn(:,:,alignLayerID) < 25e-2);

            withinReachLayerID = find(strcmp(obj.all_quality_name, "Within Reach"));
            withinReachFailMask = find(experimentDataIn(:,:,withinReachLayerID) < 1e-1);

            epsilonLayerID = find(strcmp(obj.all_quality_name, "Epsilon"));
            epsilonFailMask = find(experimentDataIn(:,:,epsilonLayerID) < 1e-1);

            volumeLayerID = find(strcmp(obj.all_quality_name, "Volume"));
            volumeFailMask = find(experimentDataIn(:,:,volumeLayerID) < 1e-13);


            failIdx = [alignFailMask;withinReachFailMask;epsilonFailMask;volumeFailMask];
            refineMask = ones(size(experimentDataIn, 1,2));
            refineMask(unique(failIdx)) = nan;

            refinedDataOut = experimentDataIn.*refineMask;


            successNumber = sum(refineMask == 1);
            failureNumber = sum(isnan(refineMask));


        end


        function [figureHandle, skewVal] = experimentPDF(obj, experimentName, varargin)
            %EXPERIMENTPDF Plot a tiled array of the experiments named as
            %the first argument
            %Options:
            %experimentPDF(experimentName)
            %experimentPDF(experimentName, measureName)
            %experimentPDF(experimentName, measureName, refineFlag)


            if length(varargin) < 2
                if isstring(varargin{1})
                    measureName = varargin{1};
                    refineFlag = 0;
                elseif isnumeric(varargin{1})
                    measureName = obj.all_quality_name;
                    refineFlag = varargin{1};
                end
            else
                measureName = varargin{1};
                refineFlag = varargin{2};
            end




            if refineFlag
                [~, rawY] = obj.extractMeasure(experimentName);
                [refinedDataOut, successNumber, failureNumber] = obj.excludeFailedGrasp(rawY{1});

                %Find the 3rd Dimension pages of interest
                [r, c] = find(obj.all_quality_name == measureName(:));
                [~, order_c] = sort(r);
                yData = refinedDataOut(:,:,c(order_c));
            else
                [~, rawY] = obj.extractMeasure(experimentName, measureName);
                yData = rawY{1};
            end



            %% Plot the PDF for all measures currently
            nMeasure = length(measureName);
            skewVal = nan(1,nMeasure);
            figureHandle = figure;
            t = tiledlayout('flow');
            title(t, experimentName, 'Interpreter', 'none');
            for m = 1:nMeasure
                nexttile
                [~, skewVal(m)] = obj.plotPDF(reshape(yData(:,:,m),[],1));
                title(measureName(m));

            end

        end

        function medianOut = transformMedian(obj, dataIn, varargin)
            %TRANSFORMMEDIAN Apply transforms to get a predictable distribution
            %For each layer of the matrix in that corresponds with the
            %measureName string array, apply the appropriate transformation
            %to get an approximately normal distribution then return the
            %median value for each column



            if isempty(varargin)
                measureName = obj.all_quality_name(1:4);
            else
                measureName = varargin{1};
            end
            nMeasure = length(measureName);

            medianOut = nan(nMeasure, size(dataIn, 2));

            for n = 1: nMeasure
                if strcmp(measureName(n), "Volume")
                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        pd = fitdist(x, "Normal");
                        medianOut(n, i) = pd.median;
                    end

                end
                if strcmp(measureName(n), "Align")

                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        pd = fitdist(x, "GeneralizedExtremeValue");
                        medianOut(n, i) = pd.median;
                    end

                end
                if strcmp(measureName(n), "Epsilon")
                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        %if isnan(median(x))
                        %   medianOut(n, i) = NaN;
                        %else
                        pd = fitdist(x, "GeneralizedExtremeValue");
                        medianOut(n, i) = pd.median;
                        %end
                    end

                end
                if strcmp(measureName(n), "COM Offset")

                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        pd = fitdist(x, "GeneralizedExtremeValue");
                        medianOut(n, i) = pd.median;
                    end
                end

                if strcmp(measureName(n), "Simulation Time")

                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        pd = fitdist(x, "Gamma");
                        medianOut(n, i) = pd.median;
                    end
                end

                if strcmp(measureName(n), "Real World Time")

                    for i = 1:size((dataIn(:,:,n)),2)
                        x = dataIn(:,i,n);
                        pd = fitdist(x, "Weibull");
                        medianOut(n, i) = pd.median;
                    end
                end



            end


        end

        function lambda_ini = boxcoxn(~, x)
            % from T. Lan on Stack OVerflow
            [m,n]=size(x);
            lambda_ini=zeros(n,1);
            for ii=1:n
                [temp,lambda_ini(ii,1)]=boxcox(x(:,ii));
            end
            fun=@(lambda)(log(det((cov(((x.^repmat(lambda',m,1)-1)./repmat(lambda',m,1))))))*m/2-(lambda-1)'*(sum(log(x)))');
            lambda=fminsearch(fun,lambda_ini);
        end



        %% Final Visualisation Functions

        function [resultTable] = extractPlotAndCostData(obj)

            rowTitle = ["Distance", "Alignment", "Distance and Alignment", "PCA"];
            nRow = length(rowTitle);

            searchMethod = ["Random", "Mean Centred"];
            nSearchMethod = length(searchMethod);

            controlMethod = ["Joint", "Location"];
            nControlMethod = length(controlMethod);

            
            qualityTitle = obj.all_quality_name(1:4);
            costTitle = ["Simulation Time", "Real World Time"];

            measureTitle = [qualityTitle, costTitle];

            nMeasure = length(qualityTitle);

            arrangeMap = {'Align_CMC_fixvar', 2, 2, 1  ; ...
                'Align_RRaP', 2, 1, 1 ; ...
                'Align_RSS', 2, 1, 2; ...
                'Align_CGMMC_fixvar', 2, 2, 2;...
                %'Align_CGMMC_varinc', 2, 2, 2;...
                %'Align_CGMMC_vardec', 2, 2, 2;...
                'IPDAlign_CMC_fixvar', 3, 2, 1  ; ...
                %'IPDAlign_CMC_varinc', 3, 2, 1 ; ...
                %'IPDAlign_CMC_vardec', 3, 2, 1 ; ...
                'IPDAlign_RRaP', 3, 1, 1;...
                'IPDAlign_RSS', 3, 1, 2;...
                'IPDAlign_CGMMC_fixvar', 3, 2, 2;...
                %'IPDAlign_CGMMC_varinc', 3, 2, 2;...
                %'IPDAlign_CGMMC_vardec', 3, 2, 2;...
                'IPD_CMC_fixvar', 1, 2, 1;...
                'IPD_RRaP', 1, 1, 1;...
                'IPD_RSS', 1, 1, 2;...
                'IPD_CGMMC_fixvar', 1, 2, 2; ...
                'PCA_CGMMC_fixvar', 4, 2, 2 ;...
                %'PCA_CGMMC_varinc', 4, 2, 2 ; ...
                %'PCA_CGMMC_vardec', 4, 2, 2 ; ...
                'PCA_CMC_fixvar', 4, 2, 1 ;...
                %'PCA_CMC_vardec', 4, 2, 1 ;...
                %'PCA_CMC_varinc', 4, 2, 1 ;...
                'PCA_RRaP', 4, 1, 1 ;...
                'PCA_RSS', 4, 1, 2};

            mapTable = cell2table(arrangeMap, "VariableNames", ["Title", "Row Index", "Search Method", "Control Method"]);

            resultCellArray = cell(size(mapTable,1),nMeasure);

            for row_i = 1:nRow
                for search_i = 1:nSearchMethod
                    for control_i = 1:nControlMethod
                        % Extract the experiments for the given row
                        rowFlag = (mapTable.("Row Index") == row_i & mapTable.("Control Method") == control_i  & mapTable.("Search Method") == search_i);
                        if any(rowFlag)
                            experimentTitle = mapTable.Title{rowFlag};

                            %Extract all data for this experiment
                            [xData, yData] = obj.extractMeasure({experimentTitle});

                            %Refine this data to exclude failed grasps
                            [refinedYData, successNumber, failureNumber] = obj.excludeFailedGrasp(yData);
                            percentRate = successNumber ./ (successNumber + failureNumber);

                            %Exclude the layer that specifies whether the
                            %grasp is within reach
                            excludeLayer = find(obj.all_quality_name == "Within Reach");
                            refinedYData(:,:,excludeLayer) = [];


                            %Find the median of each measure for these
                            %experiments
                            medianOut = obj.transformMedian(refinedYData, measureTitle);



                            for meas_i = 1:nMeasure
                                %Find the knee point for each measure of this experiment
                                resultStruct = struct('KneeNContact', [], 'KneeY', [], 'PercentSuccess', [], 'SimulationTime', [], 'RealWorldTime', []);
                                % Find the number of contacts at the knee
                                %Find the layer index if measure names are
                                %reordered
                                median_idx = find(measureTitle == qualityTitle(meas_i));
                                
                                
                                resultStruct.KneeNContact = obj.findKnee(xData{:}, medianOut(median_idx,:));

                                %Find the column index that links to that
                                %number of contacts at the knee
                                x_i = find(xData{:}==resultStruct.KneeNContact);

                                % Find the median value for the trial with
                                % that number of contacts
                                resultStruct.KneeY = medianOut(median_idx, x_i);

                                % Find the failure rate for that number of
                                % contacts
                                resultStruct.PercentSuccess = percentRate(x_i);

                                % Add the cost median for that number of
                                % contacts
                                simTime_idx = (measureTitle == "Simulation Time");
                                resultStruct.SimulationTime = medianOut(simTime_idx,x_i);

                                realWorldTime_idx = (measureTitle == "Real World Time");
                                resultStruct.RealWorldTime = medianOut(realWorldTime_idx,x_i);

                                %% Assign the values to the output cell
                                resultCellArray{find(rowFlag), meas_i} = resultStruct;

                            end


                        end
                    end
                end


            end

            dataTable = cell2table(resultCellArray, 'VariableNames', qualityTitle);

            resultTable = [mapTable, dataTable];

        end

% 
%          function [resultTable] = extractPlotData(obj)
% 
%             rowTitle = ["Distance", "Alignment", "Distance and Alignment", "PCA"];
%             nRow = length(rowTitle);
% 
%             searchMethod = ["Random", "Mean Centred"];
%             nSearchMethod = length(searchMethod);
% 
%             controlMethod = ["Joint", "Location"];
%             nControlMethod = length(controlMethod);
% 
%             measureTitle = obj.all_quality_name(1:4);
% 
%             nMeasure = length(measureTitle);
% 
%             arrangeMap = {'Align_CMC_fixvar', 2, 2, 1  ; ...
%                 'Align_RRaP', 2, 1, 1 ; ...
%                 'Align_RSS', 2, 1, 2; ...
%                 'Align_CGMMC_fixvar', 2, 2, 2;...
%                 %'Align_CGMMC_varinc', 2, 2, 2;...
%                 %'Align_CGMMC_vardec', 2, 2, 2;...
%                 'IPDAlign_CMC_fixvar', 3, 2, 1  ; ...
%                 %'IPDAlign_CMC_varinc', 3, 2, 1 ; ...
%                 %'IPDAlign_CMC_vardec', 3, 2, 1 ; ...
%                 'IPDAlign_RRaP', 3, 1, 1;...
%                 'IPDAlign_RSS', 3, 1, 2;...
%                 'IPDAlign_CGMMC_fixvar', 3, 2, 2;...
%                 %'IPDAlign_CGMMC_varinc', 3, 2, 2;...
%                 %'IPDAlign_CGMMC_vardec', 3, 2, 2;...
%                 'IPD_CMC_fixvar', 1, 2, 1;...
%                 'IPD_RRaP', 1, 1, 1;...
%                 'IPD_RSS', 1, 1, 2;...
%                 'IPD_CGMMC_fixvar', 1, 2, 2; ...
%                 'PCA_CGMMC_fixvar', 4, 2, 2 ;...
%                 %'PCA_CGMMC_varinc', 4, 2, 2 ; ...
%                 %'PCA_CGMMC_vardec', 4, 2, 2 ; ...
%                 'PCA_CMC_fixvar', 4, 2, 1 ;...
%                 %'PCA_CMC_vardec', 4, 2, 1 ;...
%                 %'PCA_CMC_varinc', 4, 2, 1 ;...
%                 'PCA_RRaP', 4, 1, 1 ;...
%                 'PCA_RSS', 4, 1, 2};
% 
%             mapTable = cell2table(arrangeMap, "VariableNames", ["Title", "Row Index", "Search Method", "Control Method"]);
% 
%             resultCellArray = cell(size(mapTable,1),nMeasure);
% 
%             for row_i = 1:nRow
%                 for search_i = 1:nSearchMethod
%                     for control_i = 1:nControlMethod
%                         % Extract the experiments for the given row
%                         rowFlag = (mapTable.("Row Index") == row_i & mapTable.("Control Method") == control_i  & mapTable.("Search Method") == search_i);
%                         if any(rowFlag)
%                             experimentTitle = mapTable.Title{rowFlag};
% 
%                             %Extract all data for this experiment
%                             [xData, yData] = obj.extractMeasure({experimentTitle});
% 
%                             %Refine this data to exclude failed grasps
%                             [refinedYData, successNumber, failureNumber] = obj.excludeFailedGrasp(yData);
%                             percentRate = successNumber ./ (successNumber + failureNumber);
% 
%                             %Find the median of each measure for these
%                             %experiments
%                             medianOut = obj.transformMedian(refinedYData);
% 
% 
% 
%                             for meas_i = 1:nMeasure
%                                 %Find the knee point for each measure of this experiment
%                                 resultStruct = struct('KneeNContact', [], 'KneeY', [], 'PercentSuccess', []);
%                                 % Find the number of contacts at the knee
%                                 %Find the layer index if measure names are
%                                 %reordered
%                                 median_idx = find(obj.all_quality_name == measureTitle(meas_i));
% 
%                                 resultStruct.KneeNContact = obj.findKnee(xData{:}, medianOut(median_idx,:));
% 
%                                 %Find the column index that links to that
%                                 %number of contacts at the knee
%                                 x_i = find(xData{:}==resultStruct.KneeNContact);
% 
%                                 % Find the median value for the trial with
%                                 % that number of contacts
%                                 resultStruct.KneeY = medianOut(meas_i, x_i);
% 
%                                 % Find the failure rate for that number of
%                                 % contacts
%                                 resultStruct.PercentSuccess = percentRate(x_i);
% 
%                                 %% Assign the values to the output cell
%                                 resultCellArray{find(rowFlag), meas_i} = resultStruct;
% 
%                             end
% 
% 
%                         end
%                     end
%                 end
% 
% 
%             end
% 
%             dataTable = cell2table(resultCellArray, 'VariableNames', measureTitle);
% 
%             resultTable = [mapTable, dataTable];
% 
%         end

        function [orderedResultTable] = orderGraspQualityPlateau(obj, mapResultTable)
            %ORDERGRASPQUALITYPLATEAU Reorder the experiments in the following
            %priority
            % 1: find median kneeNcontacts and the median success rate for a
            % given experiment across all measures
            % 2: Order the experiments by percentage success descending, then by ncontacts ascending
            % [TODO]: See what the results are if I prioritise in the opposite
            % order (number then percent)

            nRow = size(mapResultTable,1);
            medianExperimentVal = nan([nRow,4]);
            for row_i = 1:nRow
                %Find a median measure value and append to the table
                %Extract the measure data
                kneeStruct = mapResultTable(row_i,obj.all_quality_name(1:4)).Variables;
                kneeTable = struct2table(kneeStruct);
                s = summary(kneeTable);
                medianExperimentVal(row_i,:) = [s.KneeNContact.Median, s.PercentSuccess.Median, s.SimulationTime.Median, s.RealWorldTime.Median];
            end
            [sortMedian, sort_idx] = sortrows(medianExperimentVal, [2,1], {'descend', 'ascend'});
            %[sortMedian, sort_idx] = sortrows(medianExperimentVal, [1,2], {'ascend', 'descend'});

            medianTable = array2table(sortMedian, "VariableNames", ["KneeNContact Median", "PercentSuccess Median", "SimulationTime Median", "RealWorldTime Median"]);
            orderedResultTable = [mapResultTable(sort_idx,:), medianTable];

        end

        function [] = plotOrderedMedianQualityAndCost(obj, orderedResultTable)
        %PLOTORDEREDQUALITYANDCOST Show the median knee for each method
        %alongside a stacked barchart showing the different cost times
        %against a different y axis

        %Retrieve the cost for the mean number of contacts
        medianArray = orderedResultTable(:,["KneeNContact Median","PercentSuccess Median", "RealWorldTime Median", "SimulationTime Median"]).Variables;
        experimentTitle = orderedResultTable(:,"Title").Variables;
                
            kneeY = medianArray(:,1);
            percentSuccess = medianArray(:,2);
            realTime = medianArray(:,3);
            simTime = medianArray(:,4);

            plotPercentSuccess = kneeY .* percentSuccess;

            x_label = categorical(experimentTitle);
            x_tick = 1:length(x_label);
            bar_width = 0.4;

            b1 = bar(x_tick-(bar_width*0.5), kneeY, bar_width, DisplayName='Number of Contacts');
            b1.FaceColor = obj.median_colour;

            hold on

            b2 = bar(x_tick-(bar_width*0.5), plotPercentSuccess, bar_width,  DisplayName='Percentage Success');
            b2.FaceColor = obj.percent_colour;
            xError = b2.XEndPoints;
            percentlabels = string(percentSuccess);
            text(xError,plotPercentSuccess*0.5,percentlabels,'HorizontalAlignment','center',...
                'VerticalAlignment','middle', 'Rotation', 90);

            xticks(x_tick)
            xticklabels(x_label);
            xtickangle(90)
            ax = gca;
            ax.TickLabelInterpreter = 'none';

            yyaxis right
            b3 = bar(x_tick+(bar_width*0.5), medianArray(:,[3,4])', bar_width, 'stacked'); 
            b3(1).DisplayName = "Real World Time";
            b3(1).FaceColor = obj.cost_colour_rwt;
            b3(2).DisplayName = "Simulation Time";
            b3(2).FaceColor = obj.cost_colour_st;

            hold off

            legend

            title(ax, "Knee At Plateau Ordered ordered as Median across Measures")



        end

        function [] = plotOrderedResultTable(obj, orderedResultTable)

            measureStructArray = orderedResultTable(:,obj.all_quality_name(1:4)).Variables;

            kneeY = reshape([measureStructArray.KneeNContact], size(measureStructArray));
            percentSuccess = reshape([measureStructArray.PercentSuccess], size(measureStructArray));

            plotPercentSuccess = kneeY .* percentSuccess;

            x_label = categorical(orderedResultTable.Title);
            x_tick = 1:length(x_label);

            bar(x_tick, kneeY, 'red', DisplayName='Number of Contacts')


            hold on
            b = bar(x_tick, plotPercentSuccess, 'yellow', DisplayName='Percentage Success');
            xError = [b(1).XEndPoints',b(2).XEndPoints',b(3).XEndPoints',b(4).XEndPoints'];
            percentlabels = string(percentSuccess(:));
            text(xError(:),plotPercentSuccess(:)*0.5,percentlabels,'HorizontalAlignment','center',...
                'VerticalAlignment','middle', 'LineWidth', b(1).BarWidth, 'Rotation', 90);

            ax = gca;
            ax.TickLabelInterpreter = 'none';
            xticks(x_tick)
            xticklabels(x_label);
            xtickangle(90)
            hold off

            title(ax, "Knee At Plateau Ordered for all Measures")

        end

        function [] = plotOrderedMedianTable(obj, orderedResultTable)

            medianArray = orderedResultTable(:,["KneeNContact Median","PercentSuccess Median"]).Variables;

            kneeY = medianArray(:,1);
            percentSuccess = medianArray(:,2);

            plotPercentSuccess = kneeY .* percentSuccess;

            x_label = categorical(orderedResultTable.Title);
            x_tick = 1:length(x_label);

            bar(x_tick, kneeY, 'red', DisplayName='Number of Contacts')


            hold on
            b = bar(x_tick, plotPercentSuccess, 'yellow', DisplayName='Percentage Success')
            xError = b.XEndPoints;
            percentlabels = string(percentSuccess);
            text(xError,plotPercentSuccess*0.5,percentlabels,'HorizontalAlignment','center',...
                'VerticalAlignment','middle', 'LineWidth', b(1).BarWidth);

            xticks(x_tick)
            xticklabels(x_label);
            xtickangle(90)
            ax = gca;
            ax.TickLabelInterpreter = 'none';
            hold off

            legend

            title(ax, "Knee At Plateau Ordered ordered as Median across Measures")

        end

        function [] = plotResults(obj, mapResultTable)
            %Plot the complete data in a single figure

            rowTitle = ["Distance", "Alignment", "Distance and Alignment", "PCA"];
            nRow = length(rowTitle);

            searchMethod = ["Random", "Mean Centred"];
            nSearchMethod = length(searchMethod);

            controlMethod = ["Joint", "Location"];
            nControlMethod = length(controlMethod);

            %qualityTitle = obj.all_quality_name(1:4);
            qualityTitle = ["Epsilon", "COM Offset", "Align"];
            costTitle = ["Simulation Time", "Real World Time"];

            measureTitle = [qualityTitle, costTitle];

            nMeasure = length(qualityTitle);

            figure
            t = tiledlayout(nRow,nMeasure);
            title(t,'Number of Antennal Contacts at Plateau', 'FontWeight', 'bold')
            xlabel(t,'Grasp Quality Measure')
            ylabel(t,'Grasp Synthesis Method')
            for row_i = 1:nRow

                %Initialise any variables for the
                %particular row

                for meas_i = 1:nMeasure
                    xRange = 0;

                    kneeNContact = nan(nControlMethod,nSearchMethod);
                    kneeY = nan(nControlMethod,nSearchMethod);
                    kneePercent = nan(nControlMethod,nSearchMethod);

                    %Insert where the maths is for finding the appropriate
                    %rows
                    %tilePlotData = struct
                    for control_i = 1:nControlMethod
                        for search_i = 1:nSearchMethod
                            rowFlag = (mapResultTable.("Row Index") == row_i & mapResultTable.("Control Method") == control_i  & mapResultTable.("Search Method") == search_i);
                            if any(rowFlag)
                                kneeNContact(control_i, search_i) = mapResultTable(rowFlag,:).(qualityTitle(meas_i)).KneeNContact;
                                kneeY(control_i, search_i) = mapResultTable(rowFlag,:).(qualityTitle(meas_i)).KneeY;
                                kneePercent(control_i, search_i) = mapResultTable(rowFlag,:).(qualityTitle(meas_i)).PercentSuccess;
                                xRange = xRange+1;
                            end
                        end
                    end

                    %Find the index to the specific measure tile
                    %sub2ind produces linear indices which do not match
                    %next tile
                    tileIndex = sub2ind([nMeasure, nRow], meas_i, row_i);
                    ax = nexttile(tileIndex);


                    x_tick = 1:xRange;
                    x_label = categorical({controlMethod{:}});

                    knee = kneeNContact;

                    plotPercent = knee .*  kneePercent;


                    b1 = bar(x_label, knee, 'BarWidth', 1 );

                    if meas_i == 1
                        ylabel(ax, rowTitle(row_i), 'FontWeight', 'bold')
                    end
                    if row_i == nRow
                        xlabel(ax, measureTitle(meas_i), 'FontWeight','bold')
                    end

                    ax.YLim = [0 22];

                    %ax.XTickLabel =
                    b1(1).DisplayName = searchMethod(1);
                    b1(2).DisplayName = searchMethod(2);

                    xError = [b1(1).XEndPoints', b1(2).XEndPoints'];

                    contactlabels = string(knee(x_tick));
                    text(xError(x_tick),knee(x_tick),contactlabels,'HorizontalAlignment','center',...
                        'VerticalAlignment','bottom');


                    hold on
                    b2 = bar(xError(x_tick), plotPercent(x_tick), 'DisplayName', "Percent Success", 'BarWidth', 1)
                    percentlabels = string(kneePercent(x_tick));
                    b2.FaceColor = obj.percent_colour;
                    text(xError(x_tick),plotPercent(x_tick)*0.5,percentlabels,'HorizontalAlignment','center',...
                        'VerticalAlignment','middle', 'LineWidth', b1(1).BarWidth);
                    grid on
                    hold off

                end

            end

            legend

        end

        function [obj, resultTable] = completePaperPlot(obj)
            %Plot the complete data in a single figure
            [resultTable] = obj.extractPlotAndCostData;
            obj.plotResults(resultTable);
        end



        function [figureHandle, sk] = plotPDF(~, xData)
            pd = fitdist(xData, 'Normal');
            min_x = min(xData,[],'all');
            max_x = max(xData,[],'all');
            tick_size = range(xData,"all")/200;
            x_ticks = min_x:tick_size:max_x;
            pdf_y = pdf(pd, x_ticks);
            sk = skewness(xData);
            hold on
            figureHandle = plot(x_ticks, pdf_y);
            histogram(xData, 'Normalization','pdf');
            hold off

        end


        %Rewatch Experiment
        function obj = replayFromMAT(obj, file)
            [~,~,type] = fileparts(file);

            fileStruct = load(file);
            contactsTable = fileStruct.contactsTable;
            replayTable = fileStruct.replayTable;
            AntRigidBodyTree = fileStruct.antTree;

            figure(3);

            for i=1:size(replayTable,1)
                time = replayTable.Time(i);
                pose = replayTable.Pose{i,:};
                position = replayTable.Position(i,:);

                show(AntRigidBodyTree, pose ,'Parent', gca, 'Position', position, 'PreservePlot',false, 'Collisions', 'off', 'Visual', 'on', 'FastUpdate',true);

                rows = (contactsTable.Time == time);
                if any(rows)
                    idx = find(rows);
                    for p = 1:length(idx)
                        hold on;
                        collision_points = contactsTable.("Contact Location")(idx,:);
                        plot3(collision_points(1), collision_points(2), collision_points(3),'o','MarkerSize',5, 'Color', 'c')
                        hold off
                    end
                end
                pause(0.1);

            end
        end


    end
end