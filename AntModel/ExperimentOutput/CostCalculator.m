classdef CostCalculator
    %CostCalculator contains the functions and values used to evaluate the
    %costs incurred by the model in a single trial. Specifically for the
    %evaluation at the end, not for internal use.

    properties
        jointMotion
        bodyMotion
        memoryCost
        calculationTime
        weights


    end

    methods
        function obj = CostCalculator()
            %COSTCALCULATOR Construct an instance of this class
            %   Initialise the arrays to be empty
            obj.jointMotion = [];
            obj.memoryCost = 0;
            obj.calculationTime = 0;
            %Movement cost weights
            % Weights
            obj.weights.Neck = 0.5;
            obj.weights.Antenna = [0.04, 0.04, 0.02];
            obj.weights.Mandible = 0.4;
        end

        function obj = addCost(obj, costStruct)
            %ADDCOST Save any values stored in the cost struct to this
            %instance of the class

            if ~isempty(costStruct)
                 limbCalculationTime = sum(costStruct.pose.limb.time(:));
                    
                goalTimeMeasures = fieldnames(costStruct.goal.time); 
                nGoalTimeMeasures = length(goalTimeMeasures);
                sumGoalTime = 0;
                for n = 1:nGoalTimeMeasures
                    sumGoalTime = sumGoalTime + costStruct.goal.time.(goalTimeMeasures{n});

                end
                 senseEvalTime = sumGoalTime;

                 obj.calculationTime = obj.calculationTime + ...
                        limbCalculationTime + ...
                        senseEvalTime;


                 contactMemoryBytes = costStruct.memory.contact_points;
                 

                 obj.memoryCost = obj.memoryCost + contactMemoryBytes;
            end

        end

        function obj = calculateMotionCost(obj, replayTable)
            poseMovementCostStruct = obj.weightedMotionCost(obj.weights, replayTable);
            obj.jointMotion = poseMovementCostStruct.overall_cost;


        end

        function movementStruct = weightedMotionCost(~,weights, replayTable)
            %Sum the change in joint angle according to weights

            % ---------- Find the overall cost for one trial

            %time_s = 0;
            last_pose = replayTable.Pose{1,:};
            nPose = size(replayTable,1);
            nJoint = length(last_pose);

            pose_difference = nan([nPose, nJoint]);


            for i=1:nPose
                %sample_t = replayTable.Time(i);
                pose = replayTable.Pose{i,:};
                %position = replayTable.Position(i,:);

                pose_difference(i,:) = abs(pose - last_pose)';

                last_pose = pose;
                %time_s = now;
            end


            %Normalise the weights
            switch class(weights)
                case "struct"
                    weight_mat = [weights.Neck; weights.Neck; 
                        weights.Mandible; weights.Mandible;...
                        weights.Antenna(1); weights.Antenna(2); weights.Antenna(3); ...
                        weights.Antenna(1); weights.Antenna(2); weights.Antenna(3)];
                    
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

            %normalise weights [TODO] currently they are already normalised
            %(They aren't because there aren't only 3 joints, there are 3
            %joint types)
            weights_norm = weight_mat/sum(weight_mat);


            if ~isempty(pose_difference)
                %calculate the overall cost from start to end of trial
                movementStruct.duration = replayTable.Time(end) - replayTable.Time(1);
                movementStruct.overall_cost_per_joint = sum(pose_difference,1) .* weights_norm';
                movementStruct.overall_cost = sum(movementStruct.overall_cost_per_joint);
                movementStruct.joint_cost_per_second = movementStruct.overall_cost_per_joint / movementStruct.duration;
                movementStruct.overall_cost_per_second = sum(movementStruct.joint_cost_per_second);
                
            else
                warning('Pose Change over Trial not detected')
            end

            
        end


        function costTable = convertToTable(obj)
            %ConvertToTable Finalise the arrays contained within the class
            %to be in table format
            costTable = table();
            costTable.(1) = obj.jointMotion;
            costTable.(2) = obj.memoryCost;
            costTable.(3) = obj.calculationTime;
            costTable.Properties.VariableNames = {'Joint Motion', 'Memory Bytes', 'calculationTime'};
                    
            
        end
    end
end