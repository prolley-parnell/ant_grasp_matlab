classdef JointActionGen
    %JOINTACTIONGEN Class to generate joint trajectories, not using point
    %to point control, but oscillations to generate antennal sweeping
    %motions
    % %   ChangeLog: 02/11/22 - Emily Rolley-Parnell - Create new class for
    % strictly joint based control

    properties
        interval

        maxvelocities

        mean_q
        mean_weight

        search_range
        search_config
        memory_length

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

            obj.mean_q = obj.home_pose;
            obj.mean_weight = [1, 1];

            obj.search_config = RUNTIME_ARGS.SEARCH;
            obj.search_range = RUNTIME_ARGS.SEARCH.RANGE;
            obj.memory_length = RUNTIME_ARGS.ANT_MEMORY;


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

            waypoints = [maskedQ_free_pose, obj.generateSweepWP(antennaIn)];
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

        function [waypoints] = generateSweepWP(obj, antennaIn)
            %GENSWEEPWP Using the antenna control methods, generate the
            %joint waypoints for the antenna (not the incremental
            %trajectory)


            if strcmp(obj.search_config.MODE{2}, 'random')
                waypoints = obj.randomRestrict(antennaIn);
            elseif strcmp(obj.search_config.MODE{2}, 'mean')
                waypoints = obj.meanCentred(antennaIn);
            end





        end

        function [waypoints] = randomRestrict(obj, antennaIn)
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

        function [waypointArray] = meanCentred(obj, antennaIn)
            %MEANCENTRED Generate the sparse waypoints for the 3 antennal
            %joints that will be used to make the mean centred sweeping
            %trajectory
            %Extract which number of contacts to use for the mean



            %Generate the variance for each joint based on the mode and
            %joint limits
            jointVariance = obj.generateVariance(antennaIn.joint_limits, antennaIn.number);

            %sample a value for each of the (first two) joints centred
            %about te mean
            %find the mean and restrict it to the useful search range
            %[TODO] Do I want to be doing this?
            activeRange =  obj.search_range;
            limitRange = abs(antennaIn.joint_limits(:, 2) - antennaIn.joint_limits(:, 1));

            meanWorkingLimit = activeRange.*limitRange + antennaIn.joint_limits(:, 1);
            %[TODO] Check that this works given that q2 has an inverted
            %axis
            runningMeanQ = obj.mean_q(antennaIn.joint_mask==1);

            %[TODO] Check if the min/max selects based on each row
            %individually
            %Check the lower and upper working limits
            meanJointVal = tbox.applyUpperandLowerLimit(runningMeanQ(1:2, :), meanWorkingLimit(1:2,:));


            %Randomly generate joint position based on the mean and
            %variance
            jointOffset = jointVariance(1:2,:).*((2*rand([2,1])) - 1);

            newJointGoal = meanJointVal + jointOffset;

            %Ensure the values are within limits
            q12 = tbox.applyUpperandLowerLimit(newJointGoal, antennaIn.joint_limits(1:2,:));

            %also generate a sweeping motion for q3
            outPath = [q12;meanWorkingLimit(3,2)];
            sweepIn = [q12;meanWorkingLimit(3,1)];

            %compile the random base with the sweeping motion
            waypointArray = [outPath, sweepIn];

        end

        function [joint_variance] = generateVariance(obj, joint_limits, weightMask)
            %GENERATEVARIANCE Use the instruction string to give a scalar
            %value of variance based on the internal state of the model and
            %the instruction
            modeString = obj.search_config.VAR{1};
            availableModes = {'none', 'varinc', 'vardec', 'IPD'};

            modeIndex = find(contains(availableModes, modeString));

            maskedWeight = obj.mean_weight(weightMask);

            variance_scale = 1;
            variance = obj.search_config.VAR{2};
            
            if modeIndex == 2 %If variance mode increases with the number of contacts
                if maskedWeight>=2
                    variance_scale = 1 + (maskedWeight / obj.memory_length);
                end
                variance = variance_scale * obj.search_config.VAR{2};
            elseif modeIndex == 3 %If the variance mode decreases with the number of contacts
                if maskedWeight>=2
                    variance_scale = max(0.01, 1 - (maskedWeight/obj.memory_length));
                end
                variance = variance_scale * obj.search_config.VAR{2};
            elseif modeIndex == 4
                warning("IPD Not Implemented for Joint Control")
                variance = variance_scale * obj.search_config.VAR{2};
            end

            motion_range = abs(joint_limits(:,1) - joint_limits(:,2));

            joint_variance = motion_range .* variance ;

        end


        function [obj, memoryCostTime] = updateContactMemory(obj, contact_pointStruct, qIn, joint_mask_i, antennaNumber)
            %[COST] Memory cost of ActionGen class for remembering means

            tStart = tic;

            if strcmp(obj.search_config.MODE{2}, 'mean')
                %Identify which limb to find the joint mask
                %Add pose of antenna at point of contact to the mean
                obj = obj.addPoseToMean(qIn, joint_mask_i, antennaNumber);
            end




            memoryCostTime = toc(tStart);
        end


        function obj = addPoseToMean(obj, qIn, joint_mask, antenna_number)
            %ADDPOSETOMEAN Add the appropriate joint values to the mean at
            %the point of contact
            mean_weight_temp = obj.mean_weight(antenna_number);
            %Multiply the mean by the number of contacts so far
            scale_mean = obj.mean_q * mean_weight_temp;

            %Divide the mean value by the new number of samples
            sum_q = (scale_mean + qIn)/(mean_weight_temp+1);
            %Update the weighting to have a maximum based on the set memory
            %length
            obj.mean_weight(antenna_number) = min(mean_weight_temp+1, obj.memory_length);

            %Edit the mean joint values, but only the relevant ones using a
            %mask
            obj.mean_q(joint_mask==1) = sum_q(joint_mask==1);

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

