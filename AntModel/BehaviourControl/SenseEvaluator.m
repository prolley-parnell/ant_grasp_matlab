classdef SenseEvaluator
    %SENSEEVALUATOR

    properties

        mandible_max
        mode
        enforce_mand_max
        threshold
        quality
        COC

        RUNTIME_ARGS

    end

    methods
        function obj = SenseEvaluator(RUNTIME_ARGS, maxMandibleDist)
            %SENSEEVALUATOR Construct an instance of this class

            obj.mandible_max = maxMandibleDist;
            obj.mode = RUNTIME_ARGS.SENSE.MODE;
            obj.enforce_mand_max = RUNTIME_ARGS.SENSE.MAND_MAX;

            %For the variance of the centre of contacts between time steps
            obj.threshold = RUNTIME_ARGS.SENSE.THRESH;

            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            %Measure of quality of the current goal position.
            obj.quality = 0;

            %Centre of contacts, the mean value of each of the contact
            %points collected through tactile sensing
            obj.COC = struct("mean", [], "delta", []);


        end

        function [ant, goal] = check(obj, ant, sensedData)
            goal = goalStruct();
            %if ~isempty(sensedData)
            nContactPoints = length(ant.contact_points) - obj.RUNTIME_ARGS.SENSE.MINIMUM_N;
            if ~isempty(sensedData) && nContactPoints >= 0

                
                obj = obj.calcCOC(ant.contact_points);
                
                switch(obj.mode)
                    %Move towards the first pair of contacts that are closer together than the mandible distance
                    case "closest_first_pair"
                        [obj, goal] = obj.firstPair(ant.contact_points, "min");
                    case "furthest_first_pair"
                        [obj, goal] = obj.firstPair(ant.contact_points, "max");
                    case "thresh_COC"
                    case "largest_wrench_vol"
                    otherwise
                        warning("Evaluation method does not match provided functions")
                end

                mandibleContactCheck = sum(contains([sensedData(:).limb], "Mandible"));

                if mandibleContactCheck > 1
                    ant.grasp_complete = 1;
                    ant.mandible_state = 0;
                    ant.positionController.trajectory_queue = [];
                elseif mandibleContactCheck == 1
                    ant.positionController.trajectory_queue = [];
                    ant.mandible_state = 1;
                elseif mandibleContactCheck == 0
                    %TODO Move this elsewhere - Ant mandibles should open
                    %once when a new goal is given, and remain open until
                    %one mandible makes contact
                    ant.mandible_state = -1;
                end

                ant.senseEval = obj;

            end
        end

        function [obj, goal] = firstPair(obj, contactStructs, type)
            %goal = struct.empty;
            goal = goalStruct();

            %Cartesian contact points
            points = cat(1,contactStructs(:).point);
            [~, dist, idx] = obj.findInterPointDistance(points, type);

            if ~isempty(idx)
                switch type
                    case "min"
                        success_flag = dist < obj.threshold;
                    case "max"
                        success_flag = dist > obj.threshold;
                    otherwise
                        warning("Indicate min or max for threshold comparison.")
                end

                if success_flag
                    obj.threshold = dist * 1.1;
                    goal = goal.setcontact(points(idx(1:2),:));
                end
            end
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

        function [distanceMAT, distance, idx] = findInterPointDistance(obj, contact_points, type)


            distanceMAT = pdist2(contact_points, contact_points);
            distanceMAT(distanceMAT==0) = nan;

            if obj.enforce_mand_max
                distanceMAT(distanceMAT>obj.mandible_max) = nan;
            end

            switch type
                case "min"
                    distance = min(distanceMAT, [], 'all');

                case "max"
                    distance = max(distanceMAT, [], 'all');

                otherwise
                    distance = nan;
                    warning("Select the type of IPD as a char array, min or max")
            end
            [row, col] = find(distanceMAT == distance);
            if and(~isempty(row), ~isempty(col))
                idx = [row(1), col(1)];
            else
                idx = [];
            end
        end

        function dist = pairDistance(~, A, B)
            %PAIRDISTANCE
            %Find the magnitude length between point A and point B
            A2B = B - A;
            dist = sqrt(sum(A2B.^2));
        end


        function [forces] = genOpposeForces(obj, contacts)
            %Generate a pair of forces that point in the direction of
            %the other contact point as if pinched in a vice
            contactA = contacts(1,:);
            contactB = contacts(2,:);
            force_a = contactB - contactA;
            forceA_norm = force_a/vecnorm(force_a);
            forceB_norm = -forceA_norm;
            forceA = [forceA_norm,obj.force_applied];
            forceB = [forceB_norm,obj.force_applied];
            forces = [forceA;forceB];


        end


    end
end

