classdef OutputData
    %OUTPUTDATA Operating class that is used to save and store measured
    %variables from the Ant Model for an experiment across multiple trials.
    %   Used to save results to .csv or .mat files with locations defined
    %   in the RUNTIME_ARGS.
    %   ChangeLog: 22/11/22 - Emily Rolley-Parnell - Added the ability to
    %   print a cost sheet

    properties

        script_folder

        printout_path
        mat_path

        printout_format

        RUNTIME_ARGS

        contact_data
        sense_goal_data
        replay_data
       

        costClass


    end

    methods
        function obj = OutputData(RUNTIME_ARGS)
            %OUTPUTDATA Creates an instance for a given experiment and
            %creates the folders to contain the printed data, based on the
            %file location provided in RUNTIME_ARGS
            day = datestr(now, 'dd-mm-yy_HH-MM');
            caller = dbstack;
            modelDir = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
            obj.script_folder = [modelDir, '\ExperimentOutput\', caller(end).name,'\', RUNTIME_ARGS.TRIAL_NAME,'_', day];

            if RUNTIME_ARGS.RECORD.ENABLE
                obj.mat_path = [obj.script_folder, '\',RUNTIME_ARGS.RECORD.FOLDER];
                mkdir(obj.mat_path)

            else
                obj.mat_path = [];
            end

            if RUNTIME_ARGS.PRINTOUT.ENABLE

                obj.printout_format = RUNTIME_ARGS.PRINTOUT.FORMAT;
                obj.printout_path = [obj.script_folder, '\', RUNTIME_ARGS.PRINTOUT.FOLDER];
                mkdir(obj.printout_path)
            else
                obj.printout_format = [];
                obj.printout_path = [];
            end


            obj.RUNTIME_ARGS = RUNTIME_ARGS;
            obj.contact_data = {};
            obj.sense_goal_data = {};
            obj.replay_data = {};
            obj.costClass = CostCalculator();

        end

        function obj = saveTrial(obj, trial_number, antTree, tocTime)
            %saveTrialMAT Writes the data collected from the past trial to
            %a MATLAB MAT file
            % Inputs:
            % tocTime: Real world time taken from start to end of trial
            
            %Compile the sensory data
            contactsTable = cell2table(vertcat(obj.contact_data{:}));
            contactsTable.Properties.VariableNames =  {'Time', 'Contact Location', 'Surface Normal', 'Contact Limb'};

            %Compile the pose for the whole trial
            replayTable = cell2table(vertcat(obj.replay_data{:}));
            replayTable.Properties.VariableNames = {'Time', 'Pose', 'Position'};

            
            %Compile the different goals evaluated during the trial
            senseGoalTable = vertcat(obj.sense_goal_data{:});      

            obj.costClass = obj.costClass.calculateMotionCost(replayTable);

            %Make a copy of the cost summary table
            costSummaryTable = obj.costClass.convertToTable(tocTime, RUNTIME_ARGS.RATE, );

            %Rename the runtime args for no particular reason
            setupRuntimeArgs = obj.RUNTIME_ARGS;

            %If you provided a destination then save a mat file for every trial with all the set up
            %details
            if ~isempty(obj.mat_path)
                save([obj.mat_path, '\trial_',int2str(trial_number), '.mat'], 'contactsTable', 'senseGoalTable', 'replayTable', 'costSummaryTable', 'antTree', 'setupRuntimeArgs')
            end

            % If you want the experiment saved in a written format
            if ~isempty(obj.printout_path)

                filename = [obj.printout_path, '\trial_',int2str(trial_number)];

                switch obj.printout_format
                    case "spreadsheet"
                        WriteMode = 'overwritesheet';
                    otherwise
                        WriteMode = 'overwrite';
                end

                writetable(contactsTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Contact Points', 'WriteMode', WriteMode)
                writetable(senseGoalTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Sensed Goal', 'WriteMode', WriteMode)
                writetable(replayTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Model Pose and Positions', 'WriteMode', WriteMode)
                writetable(costSummaryTable, filename, 'FileType', obj.printout_format, 'Sheet', 'Trial Costs', 'WriteMode', WriteMode)
            end

            obj.sense_goal_data = {};
            obj.replay_data = {};
            obj.contact_data = {};
         
            obj.costClass = CostCalculator();

        end



        function obj = addTimeStep(obj, time, contactStructArray, pose, position, goalObj, costStruct)

            obj = obj.addPose(time, pose, position);
            obj = obj.addSensedData(time, contactStructArray);
            obj = obj.addGoal(time, goalObj);
            obj.costClass = obj.costClass.addCost(costStruct);
        end




        function obj = addSensedData(obj, time, contactStructArray)

            if ~isempty(contactStructArray)
                for i = 1:size(contactStructArray,1)
                    obj.contact_data{end+1} = {time, contactStructArray(i).point, contactStructArray(i).normal, contactStructArray(i).limb};
                end

            end
        end

        function obj = addPose(obj, time, pose, position)

            obj.replay_data{end+1} = {time, pose, position};

        end

        function obj = addGoal(obj, currentTime, goalObj)
            if ~goalObj.isempty()
                goalObj = goalObj.settime(currentTime);
                obj.sense_goal_data{end+1} = goalObj.convert2table();
            end
        end

        function obj = replayFromMAT(obj, file)
            [~,~,type] = fileparts(file);
            switch type
                case '.mat'
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



                otherwise
                    warning("Please use the '.mat' file for replay, filetype is %s", type)



            end

        end

        function summaryTable = evaluateTrialGoalMat(~, file)
            %Sum the change in joint angle according to weights
            %If plot = True, then show the cumulative cost across the trial
            %Print to file a calculation summary of:
            %Trial, Overall cost, average cost per second

            trial = [];

            [path,name,type] = fileparts(file);


            if ~strcmp(type, '.mat')
                warning("File type was %s, it must be '.mat'", type)
                return
            end

            fileStruct = load(file);
            try
            senseGoalTable = fileStruct.senseGoalTable;

            catch
                warning("replayTable not found")
                return
            end

            %Check if the trial is named in the table
            if isfield(senseGoalTable, 'Trial')
                trial = senseGoalTable.Trial{:};
            else
                name_cells = split(name, '_');
                I = find(strcmp(name_cells, 'trial'));
                trial = name_cells{I+1};
            end

            % ---------- Find the overall cost for one trial

            time = [];
            volume = [];
            epsilon = [];
            for i=1:size(senseGoalTable,1)
                time(i) = senseGoalTable.Time(i);
                volume(i) = senseGoalTable.Volume(i);
                epsilon(i) = senseGoalTable.Epsilon(i);
                dist(i) = senseGoalTable.MP_2_COM_Offset(i);
            end



            summaryTable = cell2table({time, volume, epsilon, dist});
            summaryTable.Properties.VariableNames = {'Time', 'Volume', 'Epsilon', 'MP_2_COM_Offset'};


        end


        function summaryTable = evaluateTrialMotionMat(~, file, weights)
            %Sum the change in joint angle according to weights
            %If plot = True, then show the cumulative cost across the trial
            %Print to file a calculation summary of:
            %Trial, Overall cost, average cost per second

            trial = [];
            overall_cost = [];
            pose_difference = [];

            [path,name,type] = fileparts(file);


            if ~strcmp(type, '.mat')
                warning("File type was %s, it must be '.mat'", type)
                return
            end

            fileStruct = load(file);
            try
            replayTable = fileStruct.replayTable;

            catch
                warning("replayTable not found")
                return
            end

            %Check if the trial is named in the table
            if isfield(replayTable, 'Trial')
                trial = replayTable.Trial{:};
            else
                name_cells = split(name, '_');
                I = find(strcmp(name_cells, 'trial'));
                trial = name_cells{I+1};
            end

            % ---------- Find the overall cost for one trial

            time = 0;
            last_pose = replayTable.Pose{1,:};



            for i=1:size(replayTable,1)
                now = replayTable.Time(i);
                pose = replayTable.Pose{i,:};
                position = replayTable.Position(i,:);

                pose_difference(:,i) = abs(pose - last_pose);

                last_pose = pose;
                time = now;
            end




            %Normalise the weights
            switch class(weights)
                case "struct"
                    weight_mat = [weights.Neck; weights.Neck; 
                        weights.Mandible; weights.Mandible;...
                        weights.Antenna; weights.Antenna; weights.Antenna; ...
                        weights.Antenna; weights.Antenna; weights.Antenna];
                    
                case "double"
                    if length(weights) == 10
                        weight_mat = weights;
                    else
                        warning("Weights size expected 10x1, instead it is %d x %d - setting to 1", size(weights,1), size(weights,2))
                        weight_mat = ones([10,1]);

                    end

                otherwise
                    warning("Weights are an unexpected file type - setting to 1")
                    weight_mat = ones([10,1]);

            end

            %normalise weights
            weights_norm = weight_mat/sum(weight_mat);


            if ~isempty(pose_difference)
                %calculate the overall cost from start to end of trial
                duration = replayTable.Time(end) - replayTable.Time(1);
                overall_cost_per_joint = sum(pose_difference,2)' .* weights_norm';
                overall_cost = sum(overall_cost_per_joint);
                joint_cost_per_second = overall_cost_per_joint / duration;
                overall_cost_per_second = sum(joint_cost_per_second);

            end


            summaryTable = cell2table({str2double(trial),duration, overall_cost, overall_cost_per_second, overall_cost_per_joint, joint_cost_per_second, weights_norm});
            summaryTable.Properties.VariableNames = {'Trial','Duration', 'Overall Weighted Cost', 'Overall Weighted Cost Per Second', 'Weighted Cost Per Joint', 'Weighted Joint Cost Per Second', 'Weights'};


        end

%         function [resultsTable, summaryOut] = evaluateFolder(obj, folder, weights)
% 
%             resultsTable = [];
%             fileStruct = dir([folder, '\*.mat']);
% 
%             for idx = 1:length(fileStruct)
%                 matPath = [folder, '\', fileStruct(idx).name];
%                 trialMotionTable = obj.evaluateTrialMotionMat(matPath, weights);
%                 trialGoalTable = obj.evaluateTrialGoalMat(matPath);
% 
%                 trialResults = [trialMotionTable, trialGoalTable];
%                 resultsTable = [resultsTable ; trialResults];
%             end
% 
%             %Order by trial number
%             resultsTable = sortrows(resultsTable,1);
% 
%             summaryOut = varfun(@mean, resultsTable,"InputVariables",{'Duration','Overall Weighted Cost','Overall Weighted Cost Per Second','Weighted Cost Per Joint','Weighted Joint Cost Per Second', 'Volume', 'Epsilon', 'MP_2_COM_Offset'});
% 
%         end



    end
end


