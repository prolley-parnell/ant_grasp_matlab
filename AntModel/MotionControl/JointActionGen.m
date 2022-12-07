classdef JointActionGen
    %JOINTACTIONGEN Class to generate joint trajectories, not using point
    %to point control, but oscillations to generate antennal sweeping
    %motions
    % %   ChangeLog: 02/11/22 - Emily Rolley-Parnell - Create new class for
    % strictly joint based control

    properties
        interval

        maxvelocities

        search_range
        search_config
        refineSearch

        home_pose

    end

    methods
        function obj = JointActionGen(RUNTIME_ARGS)
            %SAMPLEACTIONGEN A Class to generate the interpolated waypoints
            %for a path of action for an antenna
            obj.interval = RUNTIME_ARGS.RATE;
            obj.maxvelocities = ones([10,1]) * deg2rad(5) / obj.interval;
            %Make the scape pedicel joint have a higher speed limit
            obj.maxvelocities([7 10]) = deg2rad(9) / obj.interval;

            obj.home_pose = RUNTIME_ARGS.ANT_POSE;

            obj.search_config = RUNTIME_ARGS.SEARCH_SPACE;
            obj.search_range = RUNTIME_ARGS.SEARCH_SPACE.JOINT.RANGE;

            if strcmp(RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE, 'IG')
                obj.refineSearch = InformationGain(RUNTIME_ARGS);
            else
                obj.refineSearch = struct.empty;
            end

        end



        function [antennaOut, antennaTrajectoryTime] = loadAntennaTrajectory(obj, antennaIn, ~, ~)
            %loadAntennaTrajectory
            antennaOut = antennaIn;
            
            %% [COST] Start time to calculate antenna trajectory
            trajStartTime = tic;
            trajectoryQueue = obj.generateJointTrajectory(antennaIn);
            
            antennaTrajectoryTime = toc(trajStartTime);
            % [COST] End time to calculate antenna trajectory

            antennaOut.trajectory_queue = trajectoryQueue;

        end


        function [qTrajectory] = generateJointTrajectory(obj, antennaIn)
            %generateJointTrajectory given the method of sweeping, generate
            %the next full joint trajectory path for the antenna given the
            %velocity limits for each joint

            maskedQ_free_pose = antennaIn.free_pose;

            jointIDArray = find(antennaIn.joint_mask == 1);
            qTrajectory = maskedQ_free_pose;

            waypoints = [maskedQ_free_pose, obj.genSweepWP(antennaIn)];
            nPath = size(waypoints,2)-1;
            for i = 1:nPath
                waypoint_i = waypoints(:,i:i+1);
                subTrajectory = obj.trapVelGen(waypoint_i, jointIDArray);
                qTrajectory = cat(2, qTrajectory, subTrajectory);
            end

            qTrajectory(:,1) = [];


        end

        function [trajectory, velocity] = trapVelGen(obj, jointWaypoints, jointID)
            %Check whether the start and end position are different
            nJoint = size(jointWaypoints,1);
            trajectoryCell = cell([nJoint,1]);
            velocityCell = cell([nJoint,1]);

            roundWP = round(jointWaypoints,2);
            equalFlag = roundWP(:,1) == roundWP(:,2);

            if all(equalFlag)
                trajectory = [];
                velocity = [];
            else
                nSample = nan(nJoint,1);
                for n=1:nJoint
                    if ~equalFlag(n)
                        [q, qd, ~, t] = trapveltraj(jointWaypoints(n,:),500,...
                            PeakVelocity=obj.maxvelocities(jointID(n)));
                        div_t = t / obj.interval;
                        round_div_t = round(div_t);
                        [~, i_first_unique] = unique(round_div_t, 'stable');

                        trajectoryCell{n} = [q(i_first_unique), q(end)];
                        velocityCell{n} = [qd(i_first_unique), qd(end)];
                        nSample(n) = length(trajectoryCell{n});
                    end
                end

                trajLength = max(nSample);
                trajectory = ones([nJoint, trajLength]).*jointWaypoints(:,end);

                velocity = zeros(size(trajectory));
                for m=1:nJoint
                    if ~isempty(trajectoryCell{m})
                        trajectory(m,[1:nSample(m)]) = trajectoryCell{m};
                        velocity(m,[1:nSample(m)]) = velocityCell{m};
                    end
                end
            end
        end

        function [waypoints] = genSweepWP(obj, antennaIn)
            activeRange =  obj.search_range;
            limitRange = abs(antennaIn.joint_limits(:, 2) - antennaIn.joint_limits(:, 1));

            restrictJointLimit = activeRange.*limitRange + antennaIn.joint_limits(:, 1);

            minAB = restrictJointLimit(1:2, 1);
            maxAB = restrictJointLimit(1:2, 2);
            r = @(a, b, set) (a + (b-a).*set);
            randomScale = rand([2,1]);
            jointAB = r(minAB,maxAB,randomScale);

            outPath = [jointAB;restrictJointLimit(3,2)];
            sweepIn = [jointAB;restrictJointLimit(3,1)];

            waypoints = [outPath, sweepIn];


        end



        function [obj, memoryCostStruct] = updateContactMemory(obj, contact_pointStruct)
            memoryCostStruct = struct.empty;
            %[COST] Memory cost of ActionGen class for remembering means
            %[TODO]
            if strcmp(obj.search_config.JOINT.MODE, "GM")
                obj = obj.updateGM(contact_pointStruct);
            end
            if ~isempty(obj.refineSearch)
                obj.refineSearch = obj.refineSearch.setContactMemory(contact_pointStruct);
            end

        end

        function obj = updateGM(obj, contact_pointStruct)

            points = cat(1,contact_pointStruct(:).point);
            n = size(points,1);
            d = size(points,2);

            %mu = points;

            I = eye(d,d);
            p = ones([1, n])/n;
            %variance of the distribution around each point
            if strcmp(obj.search_config.JOINT.VAR, "IPD")
                if n > 1
                    distanceMAT = pdist2(points, points);
                    distanceMAT(distanceMAT==0) = nan;
                    minDist = min(distanceMAT);
                    maxDist = max(distanceMAT);

                    p = maxDist/sum(maxDist);

                    dist = minDist;

                else
                    dist = 1;
                end
                sigma = I.* reshape(dist, [1 1 n]);

            else
                if ~strcmp(class(obj.search_config.JOINT.VAR), "double")
                    warning("The set variance is not a valid value - overwrite to 1")
                    obj.search_config.SAMPLE.VAR = 1;
                end

                variance = obj.search_config.SAMPLE.VAR;


                sigma = repmat(I*variance, [1 1 n]);
            end

            gmObj = gmdistribution(points,sigma,p);

            obj.search_range = gmObj;
        end


        function outputTrajectory = loadNeckTrajectory(obj, neckIn, qIn, goalStructIn)

            [yawOut,global2goalR] = tbox.findGoalrotmat(goalStructIn);

            % -- Apply the difference in rotation to the current pose of
            % the head

            sourcebody = neckIn.base_name;
            targetbody = neckIn.end_effector;
            currentPoseTF = getTransform(neckIn.full_tree, qIn, targetbody, sourcebody);
            base2EETF = getTransform(neckIn.full_tree, homeConfiguration(neckIn.full_tree), targetbody, sourcebody);


            goalPose = rotm2tform(global2goalR) * base2EETF;


            % -- interpolate between the current pose and the rotated pose
            tSamples = 0:0.05:1;
            [waypoints,~,~] = transformtraj(currentPoseTF,goalPose,[0 1],tSamples);

            outputTrajectory = obj.trajfromWP(neckIn, waypoints, qIn);

        end



        function [mandibleOut, successFlag] = loadMandibleTrajectory(obj, mandibleIn, qIn)
            mandibleOut = mandibleIn;
            try
                %Stock the trajectory variable with the joint positions to
                %match the motion direction

                if and(abs(mandibleIn.motion_state), ~mandibleIn.collision_latch)
                    if mandibleIn.motion_state < 0
                        %opening
                        col = 2;
                    else
                        %closing
                        col = 1;
                    end


                    %Find the maximum open joint values
                    goal_joint_val = mandibleIn.joint_limits(:,col);
                    current_joint_val = qIn(mandibleIn.joint_mask==1);

                    points = [current_joint_val , goal_joint_val];
                    numSamples = 10;

                    [q, ~, ~, ~, ~] = trapveltraj(points, numSamples, 'PeakVelocity', obj.maxvelocities(mandibleIn.joint_mask==1)*obj.interval);
                    %Remove the duplicate position
                    q(:,1) = [];
                    mandibleOut.trajectory_queue = q;

                else
                    mandibleOut.trajectory_queue = [];

                end

                successFlag = 1;

            catch

                successFlag = 0;

            end
        end
    end
end

