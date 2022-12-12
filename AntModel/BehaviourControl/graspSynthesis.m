classdef graspSynthesis
    %GRASPSYNTHESIS Given the contact points, select a grasp location
    %Changelog - created 03/11/2022
    properties

        mandible_max
        synth_method
        enforce_mand_max

        quality
        COC

        RUNTIME_ARGS

    end

    methods
        function obj = graspSynthesis(RUNTIME_ARGS, maxMandibleDist)
            %GRASPSYNTHESIS Construct an instance of this class

            obj.mandible_max = maxMandibleDist;
            obj.enforce_mand_max = RUNTIME_ARGS.SENSE.MAND_MAX;

            methodArg = RUNTIME_ARGS.SENSE.MODE; %Cell array of the different qualities used to select a grasp

            nMethodArg = length(methodArg);
            availableMethod = {'dist', 'align'};
            nAvailableMethod = length(availableMethod(:));
            measureFlag = zeros([nMethodArg, nAvailableMethod]);
            for t=1:nMethodArg
                measureFlag(t,:) = strcmp(methodArg{t}, availableMethod);
                if ~any(measureFlag(t,:))
                    warning('Grasp Synthesis Measure "%s" not implemented', methodArg{t})
                end
            end
            obj.synth_method = any(measureFlag,1);

            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            %Measure of quality of the current goal position.
            obj.quality = RUNTIME_ARGS.SENSE.THRESH;

            %Centre of contacts, the mean value of each of the contact
            %points collected through tactile sensing
            obj.COC = struct("mean", [], "delta", []);


        end

        function [antOut, goalOut, goalCostStruct] = check(obj, antIn, sensedData)
            %CHECK Use the methods defined to evaluate the contact point data pairs and select a goal

            goalCostStruct = struct(); %Define the table used to store cost info
            %% [COST] Start exhaustive grasp search
            senseTStart = tic; 
            goalOut = goalStruct();
            antOut = antIn;
            
            %Deal with exceptions (no contact points, or no grasp synthesis method)
            nContactThresh = length(antIn.contact_points) - obj.RUNTIME_ARGS.SENSE.MINIMUM_N;
            if isempty(sensedData)
                %[COST] End time if no new data has been collected
                senseEvalTEnd = toc(senseTStart);
                goalCostStruct.time.sense_eval = senseEvalTEnd;
                return
            elseif nContactThresh < 0
                %[COST] Record time up to point where not enough contacts
                %have been gathered to gen goal
                senseEvalTEnd = toc(senseTStart);
                goalCostStruct.time.sense_eval = senseEvalTEnd;
                return
            else
                nMeasures = length(obj.synth_method);
                if nMeasures < 1
                    warning("No grasp synthesis method selected, goal cannot be generated");
                    %[COST] End time measure if no grasp selection method
                    %is selected
                    senseEvalTEnd = toc(senseTStart);
                    goalCostStruct.time.sense_eval = senseEvalTEnd;
                    return
                end
            end
            %Cartesian contact points
            cartPointArray = cat(1,antIn.contact_points(:).point);
            nContactPoint = length(antIn.contact_points);
            distanceMat = ones(nContactPoint);
            mandMaxFlag = ones(nContactPoint);
            alignMeasure = ones(nContactPoint);

            %For each goal, get the information gain measure
            if obj.synth_method(1) %Distance
                [distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");
                %Normalise the distance mat to be between 0 and 1
                %distMeasure = rescale(distanceMat);
            end
            if obj.enforce_mand_max
                if ~obj.synth_method(1)
                    [distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");
                end
                mandMaxFlag = obj.mandibleLimit(distanceMat);
            end
            if obj.synth_method(2) %Alignment
                [alignMeasure, ~, ~] = obj.findInterPointGraspAlign(antIn.contact_points);
            end


            [bestQuality, goalIndex] = max(distanceMat.*mandMaxFlag.*alignMeasure,[],'all');

            %Only update to a new goal if the goal is 10% better than the
            %last
            %if obj.quality*1.1 < bestQuality
            [a, b] = ind2sub([nContactPoint, nContactPoint], goalIndex);
            
            %
            senseEvalTEnd = toc(senseTStart);
            goalCostStruct.time.sense_eval = senseEvalTEnd;
            % [COST] End exhaustive grasp search

            %%
            [goalOut, setContactTime] = goalOut.setcontact(antIn.contact_points([a,b]));
            obj.quality = bestQuality;
            
            %[COST] Save the time taken to save the selected contact points
            goalCostStruct.time.contact_set = setContactTime;
            
            %end
            antOut = obj.mandibleMotionStateMachine(antIn, sensedData);

            antOut.graspGen = obj;


        end

        function [flagArray] = mandibleLimit(obj, distanceArray)

            flagArray = ones(size(distanceArray));
            flagArray(distanceArray>obj.mandible_max) = 0;

        end

        function antOut =  mandibleMotionStateMachine(~, antIn, sensedData)
            mandibleContactCheck = [];
            for l = 1:length(antIn.limbs)
                if contains(antIn.limbs{l}.name, "Mandible")
                    mandibleContactCheck(end+1) = antIn.limbs{l}.collision_latch;
                end
            end
            %mandibleContactCheck = sum(contains([sensedData(:).limb], "Mandible"));
            antOut = antIn;
            if mandibleContactCheck > 1
                antOut.grasp_complete = 1;
                antOut.mandible_state = 0;
                antOut.positionController.trajectory_queue = [];
            elseif mandibleContactCheck == 1
                antOut.positionController.trajectory_queue = [];
                antOut.mandible_state = 1;
            end
        end



        function [alignMat, bestAlign, idx] = findInterPointGraspAlign(obj, contactStruct)
            %findInterPointGraspAlign find the alignment at a point of contact
            %with the surface vector for a given opposing contact

            pointArray = cat(1,contactStruct(:).point);
            normArray = cat(1,contactStruct(:).normal);
            nContact = size(pointArray,1);
            alignMat = nan([nContact, nContact]);
            idx = [];

            %[distanceMAT, ~, ~] = obj.findInterPointDistance(pointArray, type);
            for n = 1:nContact
                for m = 1:nContact
                    if n~= m
                        contactPair = pointArray([n, m],:);
                        forcePair = obj.genOpposeForces(contactPair);
                        normalPair = normArray([n,m],:);

                        alignPair = min(tbox.findSurfNormAlign(normalPair, forcePair));
                        alignMat(n,m) = alignPair;
                        alignMat(m,n) = alignPair;
                    end

                end


            end
            bestAlign = max(alignMat,[], 'all');
            [idx(:,1), idx(:,2)] = find(alignMat == bestAlign);

        end
        function obj = calcCOC(obj, contactStruct)
            new_COC = mean(cat(1,contactStruct(:).point),1);
            if ~isempty(obj.COC.mean)
                delta = sqrt(sum((obj.COC.mean - new_COC).^2));
                obj.COC.delta = delta;

                if obj.RUNTIME_ARGS.PLOT.ENABLE(2)
                    figure(2)
                    hold on;
                    plot3([obj.COC.mean(1) ; new_COC(1)],[obj.COC.mean(2) ; new_COC(2)] ,[obj.COC.mean(3); new_COC(3)], '+--')
                    hold off;
                end
            end

            obj.COC.mean = new_COC;

        end

        function [distanceMAT, distance, idx] = findInterPointDistance(~, contact_points, type)


            distanceMAT = pdist2(contact_points, contact_points);
            distanceMAT(distanceMAT==0) = nan;


            switch type
                case "min"
                    distance = min(distanceMAT, [], 'all');

                case "max"
                    distance = max(distanceMAT, [], 'all');

                otherwise
                    distance = nan;
                    %Min or Max not selected - no problem
            end
            [row, col] = find(distanceMAT == distance);
            if and(~isempty(row), ~isempty(col))
                idx = [row(1), col(1)];
            else
                idx = [];
            end
        end

        function [forces] = genOpposeForces(~, contacts)
            %Generate a pair of forces that point in the direction of
            %the other contact point as if pinched in a vice
            contactA = contacts(1,:);
            contactB = contacts(2,:);
            force_a = contactB - contactA;
            forceA_norm = force_a/vecnorm(force_a);
            forceB_norm = -forceA_norm;
            forces = [forceA_norm;forceB_norm];

        end

    end
end

