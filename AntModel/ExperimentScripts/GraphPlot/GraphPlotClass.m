%% Class to process experiment data and re-run graphs on command
% Created 23/01/23
classdef GraphPlotClass

    properties
        experimentPath = {};
        experimentDataStruct


    end

    methods
        function obj = GraphPlotClass(~)
            %GraphPlotClass Constructor
            obj.experimentDataStruct = struct('Title', {}, ...
                'nContact', [], ...
                'trialData', struct('contactNumber', [], 'senseGoalTable', table.empty, 'costSummaryTable', table.empty, 'nTrial', []));

        end

        %% Final Visualisation Functions

        function obj = completePaperPlot(obj)
            %Plot the complete data in a single figure


            rowTitle = ["Distance", "Alignment", "Distance and Alignment", "PCA"];
            nRow = length(rowTitle);

            searchMethod = ["Random", "Mean Centred"];
            nSearchMethod = length(searchMethod);

            controlMethod = ["Joint", "Location"];
            nControlMethod = length(controlMethod);

            %colTitle = ["Volume", "Epsilon", "normAlign"];
            colTitle = ["Volume"];
            nCol = length(colTitle);

            arrangeMap = {'Align_CMC_fixvar', 2, 2, 1  ; ...
                'Align_RRaP', 2, 1, 1 ; ...
                'Align_RSS', 2, 1, 2; ...
                'IPDAlign_CMC_fixvar', 3, 2, 1  ; ...
                %'IPDAlign_CMC_varinc', 3, 2, 1 ; ...
                %'IPDAlign_CMC_vardec', 3, 2, 1 ; ...
                'IPDAlign_RRaP', 3, 1, 1;...
                'IPDAlign_RSS', 3, 1, 2;...
                'IPD_CMC_fixvar', 1, 2, 1;...
                'IPD_RRaP', 1, 1, 1;...
                'IPD_RSS', 1, 1, 2;...
                'PCA_CGMMC_fixvar', 4, 2, 2 ;...
                %'PCA_CGMMC_varinc', 4, 2, 2 ; ...
                %'PCA_CGMMC_vardec', 4, 2, 2 ; ...
                'PCA_CMC_fixvar', 4, 2, 1 ;...
                %'PCA_CMC_vardec', 4, 2, 1 ;...
                %'PCA_CMC_varinc', 4, 2, 1 ;...
                'PCA_RRaP', 4, 1, 1 ;...
                'PCA_RSS', 4, 1, 2};

            mapTable = cell2table(arrangeMap, "VariableNames", ["Title", "Row Index", "Search Method", "Control Method"]);

            t = tiledlayout(nRow,nCol);

            meas_i = 1;
            row_i = 1;

            for row_i = 1:nRow


                for meas_i = 1:nCol

                    %Find the index to the specific measure tile
                    tileIndex = sub2ind([nRow,nCol], row_i, meas_i);
                    nexttile(tileIndex);

                    if meas_i == 1
                        ylabel(rowTitle(row_i))
                    end
                    if row_i == 1
                        xlabel(colTitle(meas_i))
                    end


                    %Loop through the (4) different experiments that fill that tile
                    xRange = 0;
                    knee = nan(nControlMethod,nSearchMethod);
                    kneeY = nan(nControlMethod,nSearchMethod);
                    kneeIQR = nan(nControlMethod,nSearchMethod);
                    for cont_i = 1:nControlMethod
                        for search_i = 1:nSearchMethod
                            %Find the experiment title
                            rowFlag = (mapTable.("Row Index") == row_i & mapTable.("Control Method") == cont_i  & mapTable.("Search Method") == search_i);
                            if any(rowFlag)
                                experimentTitle = mapTable.Title{rowFlag};

                                %Extract the data for this column measure
                                [xData, yData] = obj.extractMeasure({experimentTitle}, colTitle(meas_i));
                                [medianVal, IQRVal] = obj.findMedAndIQR(yData);

                                %Find the knee point of this experiment
                                knee(cont_i,search_i) = obj.findKnee(xData{:}, medianVal);

                                kneeY(cont_i,search_i) = medianVal(xData{:}==knee(cont_i,search_i));
                                kneeIQR(cont_i,search_i) = IQRVal(xData{:}==knee(cont_i,search_i));

                                xRange = xRange+1;
                            end

                        end
                    end
                    x_tick = 1:xRange;
                    x_label = categorical({controlMethod{:}});
                    
                    b = bar(x_label, knee);
                    b(1).DisplayName = searchMethod(1);
                    b(2).DisplayName = searchMethod(2);
                    
                    xError = [b(1).XEndPoints', b(2).XEndPoints'];
                    
                    labels = string(knee(x_tick));
                    text(xError(x_tick),knee(x_tick),labels,'HorizontalAlignment','center',...
    'VerticalAlignment','top')


                    hold on
                    errorbar(xError(x_tick), knee(x_tick), kneeIQR(x_tick)/2, 'o', 'DisplayName', "IQR")
                end
                
            end
            legend
            hold off
        end

        %% Functions to load MAT files

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



        function [experimentTitle, nContacts, trialNumber] = addressFromPath(~, MATPath)
            folderCell = split(MATPath, '\');

            matName = split(folderCell{end}, {'_', '.'});
            trialNumber = str2num(matName{2});

            contactsFileName = split(folderCell{end-2}, {'_', '.'});
            nContacts = str2num(contactsFileName{1});

            experimentTitle = folderCell{end-3};

        end

        %%
        function [xData, yData] = extractMeasure(obj, experimentName, measureName)
            %% For a given experiment across all contact values, find the point where the measure plateaus

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


                goalQualityNameFlag = ismember(measureName, obj.experimentDataStruct(experimentIdx(i)).trialData(1).senseGoalTable.Properties.VariableNames);
                costNameFlag = ismember(measureName, obj.experimentDataStruct(experimentIdx(i)).trialData(1).costSummaryTable.Properties.VariableNames);

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

        function [medianVal, IQRVal] = findMedAndIQR(obj, dataIn)
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
            y_s = smooth(medianSetIn,'sgolay', 1)';

            %2: Normalise to a unit square
            x_sn = rescale(x_s);
            y_sn = rescale(y_s);

            %3: Calculating the difference curve changes
            x_d = x_sn;
            y_d = y_sn - x_sn;

            %plot(x_d, y_d)

            %3.5 Horizontal Axis for cross comparison
            diff = [x_d(end) - x_d(1), y_d(end) - y_d(1)];

            %4: Calculating the local maxima of the difference curve
            y_lm_flag = islocalmax(y_d);

            y_lm = y_d(y_lm_flag==1);
            x_lm = x_d(y_lm_flag==1);

            %plot(x_d,y_d,x_lm,y_lm,'r*')

            %5: Calculating threshold for each local maximum in the difference curve
            %S is sensitivity
            S = 0.9;
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
        end



        function dataOut = transformData(obj, dataIn)

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




        function figureHandle = plotPDF(~, xData)
            pd = fitdist(xData, 'Normal');
            min_x = min(xData,[],'all');
            max_x = max(xData,[],'all');
            tick_size = range(xData,"all")/200;
            x_ticks = min_x:tick_size:max_x;
            pdf_y = pdf(pd, x_ticks);
            sk = skewness(xData)
            figureHandle = plot(x_ticks, pdf_y);
            histfit(xData)

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