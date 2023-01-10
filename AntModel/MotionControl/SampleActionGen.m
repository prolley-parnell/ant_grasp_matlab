classdef SampleActionGen
    %SAMPLEACTIONGEN Used to generate trajectories for the joints in the
    %antennae and mandibles. Functions relate to point to point control in
    %cartesian space and use inverse kinematics

    properties
        interval

        %weights
        maxvelocities

        gik

        search_range
        search_config
        refineSearch

    end

    methods
        function obj = SampleActionGen(fullTree, RUNTIME_ARGS)
            %SAMPLEACTIONGEN A Class to generate the interpolated waypoints
            %for a path of action for an antenna
            obj.interval = RUNTIME_ARGS.RATE;
            obj.maxvelocities = ones([10,1]) * deg2rad(3) / obj.interval;
            %Make the scape pedicel joint have a higher speed limit
            obj.maxvelocities([7 10]) = deg2rad(7) / obj.interval;

            obj.search_config = RUNTIME_ARGS.SEARCH;
            obj.search_range = RUNTIME_ARGS.SEARCH.RANGE;

            if strcmp(RUNTIME_ARGS.SEARCH.REFINE.MODE, 'IG')
                obj.refineSearch = InformationGain(RUNTIME_ARGS);
            else
                obj.refineSearch = struct.empty;
            end

            %Initialise Generalized IK Solver
            obj.gik = generalizedInverseKinematics('RigidBodyTree', fullTree);

            obj.gik.ConstraintInputs = {"joint", "pose"};
            obj.gik.SolverParameters.AllowRandomRestart = false;

        end



        function [antennaOut, antennaTrajectoryTime] = loadAntennaTrajectory(obj, antennaIn, qIn, positionIn)
            %LOADANTENNATRAJECTORY generate a sampling point in cartesian space and
            % generate a joint trajectory to reach the sampling point
            %% [COST] Start time to calculate antenna trajectory
            trajStartTime = tic;
            antennaOut = antennaIn;
            if ~isempty(obj.refineSearch)
                N = obj.refineSearch.n_sample;
                goalArray = obj.randomGoalGen(N);
            else
                %Generate N random goals from the search space
                goalArray = obj.randomGoalGen(1);
            end

            %Turn the goal(s) in to trajectories
            goalPropStruct = obj.generateGoalProp(antennaIn, qIn, positionIn, goalArray);


            if ~isempty(obj.refineSearch)
                goalPropStructOut = obj.refineSearch.refine(goalPropStruct);

            else
                goalPropStructOut = goalPropStruct(1);
            end
            antennaTrajectoryTime = toc(trajStartTime);
            % [COST] End time to calculate antenna trajectory
            antennaOut.trajectory_queue = goalPropStructOut.jointPath;

        end

        function [qTrajectory] = jointTrajectory(obj, waypoints, velLimits)
            %numSamples = 10;
            numSamples = 500;
            [q, ~, ~, t] = trapveltraj(waypoints,numSamples,...
                PeakVelocity=velLimits);
            div_t = t / obj.interval;
            round_div_t = round(div_t);
            [~, i_first_unique] = unique(round_div_t, 'stable');

            qTrajectory = [q(:,i_first_unique), q(:,end)];

            %[qTrajectory, ~, ~, ~, ~] = trapveltraj(waypoints, numSamples, 'PeakVelocity', velLimits);
            %Remove the duplicate position
            %qTrajectory(:,1) = [];


        end

        function waypoints = generateWaypoints(obj, start_pt, end_pt, pattern)
            switch(pattern)
                case "spiral"
                    waypoints = obj.spiralPath(start_pt, end_pt);

                case "straight"
                    waypoints = obj.straightPath(start_pt, end_pt);

                case "curve"
                    waypoints = obj.curvePath(start_pt, end_pt);

                otherwise
                    warning("Antennal motion path style not specified")
            end

        end



        function waypoints = straightPath(~, start_pt, end_pt)


            vector = end_pt - start_pt;

            step_size = 0.07;

            waypoints(1,:) = [start_pt(1) : vector(1)*step_size : end_pt(1)];
            waypoints(2,:) = [start_pt(2) : vector(2)*step_size : end_pt(2)];
            waypoints(3,:) = [start_pt(3) : vector(3)*step_size : end_pt(3)];


        end


        function waypoints = curvePath(~, start_pt, end_pt)


            vector = end_pt - start_pt;
            distance = sqrt(sum(vector(:).^2));
            step_size = 0.1;



            t = [0:pi*step_size:pi];
            height = sin(t) * distance/4;

            waypoints = zeros([3, size(t, 2)]);

            waypoints(1,:) = [start_pt(1) : vector(1)/(length(t)-1) : end_pt(1)];
            waypoints(2,:) = [start_pt(2) : vector(2)/(length(t)-1) : end_pt(2)];
            waypoints(3,:) = [start_pt(3) : vector(3)/(length(t)-1) : end_pt(3)] + height;


        end


        function waypoints = spiralPath(~, start_pt, end_pt)

            vector = end_pt - start_pt;
            distance = sqrt(sum(vector(:).^2));
            step_size = 0.2;


            t = [0:step_size:distance];
            y = cos(t);
            z = sin(2*t);

            spiral = [t; y; z];

            waypoints = zeros(size(spiral));

            waypoints(1,:) = t + [start_pt(1) : vector(1)/(length(t)-1) : end_pt(1)];
            waypoints(2,:) = y + [start_pt(2) : vector(2)/(length(t)-1) : end_pt(2)];
            waypoints(3,:) = z + [start_pt(3) : vector(3)/(length(t)-1) : end_pt(3)];

            %plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:));



        end


        function [goalTrajProperties] = generateGoalProp(obj, antennaIn, qIn, positionIn, goalArray)
            %GENERATEGOALPROP Use the control methods for the antennae,
            %fill in the struct that includes the cartesian start and end
            %points, and the cartesian and joint trajectories.

            %Define the goal struct
            goalTrajProperties = struct('startPt', [], 'endPt', [], 'cartesianPath', [], 'jointPath', []);
                        
            %Determine whether the trajectories are generated with finding
            %joint positions for cartesian points, or finding the joint
            %position for the end point and interpolating
            trajOption = {'jointInterp', 'cartesianPath'};

            trajGenFlag = contains(trajOption, antennaIn.control_type);

            %Check for errors in assigning the mode of generating the
            %trajectory to the cartesian point.
            if all(trajGenFlag)
                warning('Too many options selected, default to jointInterp');
                trajGenMode = 'jointInterp';
                trajGenFlag = contains(trajOption, trajGenMode);
            elseif any(trajGenFlag)
                %trajGenMode = trajOption{trajGenFlag==1}
            else
                warning('No option selected for traj generation, default to cartesianPath')
                trajGenMode = 'jointInterp';
                trajGenFlag = contains(trajOption, trajGenMode);
            end

            trajGenModeIdx = find(trajGenFlag);

            nGoal = size(goalArray,1);
            for g = 1:nGoal
                goalTrajProperties(g).startPt = antennaIn.free_point;
                goalTrajProperties(g).endPt = goalArray(g,:);
                
                if trajGenModeIdx==1 %jointInterp
                        %Generate trajectory using interpolation between
                        %the joint configuration for the initial tip position
                        % and the goal point joint config
                        
                        waypointsGlobal = [antennaIn.free_point', goalArray(g,:)'];

                        %Find the joint configurations for the last free
                        %point and the goal
                        [jointWaypoints] = antennaIn.findIKforGlobalPt(waypointsGlobal);

                        %Load joint velocity limits
                        velocityLims = obj.maxvelocities(antennaIn.joint_mask==1);

                        %Generate the joint waypoints along the path that
                        %fit to the velocity limits
                        goalTrajProperties(g).jointPath = obj.jointTrajectory(jointWaypoints, velocityLims);

                        %Find the number of intermediate poses
                        nPose = size(goalTrajProperties(g).jointPath,2);

                        if ~isempty(obj.refineSearch)
                            %If the IGEF doesn't need the cartesian path of
                            %the motion for evaluating cost, then don't
                            %generate it
                            if obj.refineSearch.information_measures(2)
                                cartesianOut = nan([nPose,3]);

                                for n = 1:nPose
                                    % --- If code stops working after global
                                    % position changes --%
                                    %pose_n = antennaIn.applyMask(qIn, goalTrajProperties(g).jointPath(:,n));
                                    %cartesianOut(n,:) = tbox.findFKglobalPosition(antennaIn.full_tree, pose_n, positionIn, antennaIn.end_effector);

                                    % -- Assuming the code continues to work
                                    % properly on the local scale (as subtrees
                                    % are updated) -- %%
                                    cartesianOut(n,:) = tbox.findFKlocalPosition(antennaIn.subtree, ...
                                        goalTrajProperties(g).jointPath(:,n), ...
                                        antennaIn.end_effector);
                                end

                                goalTrajProperties(g).cartesianPath = cartesianOut;
                            end
                        end


                elseif trajGenModeIdx == 2 %cartesianPath
                        %Generate a trajectory made up of cartesian points
                        %and find the joint configurations to meet those
                        %points.

                        goalTrajProperties(g).cartesianPath = obj.generateWaypoints(antennaIn.free_point, goalArray(g,:), "curve");
                        %Find the global to local transform for the model
                        localModelTF = tbox.modelPosition2GlobalTF(positionIn);
                        %Transform waypoints from global to local frame
                        waypointsLocal = tbox.global2local(goalTrajProperties(g).cartesianPath, localModelTF);
                        %Find the joint configuration for each of the
                        %cartesian waypoints
                        goalTrajProperties(g).jointPath = obj.jointConfigFromCartesian(antennaIn, waypointsLocal, qIn);


                end

            end

        end

        function [goal] = randomGoalGen(obj, nGoal)
            %Generate an antennal goal position in the world frame

            switch class(obj.search_range)

                case "gmdistribution"
                    goal = random(obj.search_range, nGoal);

                case "double"

                    r = @(a, b, set) (a + (b-a).*set);

                    init_val = rand([nGoal, 3]);


                    X = r(obj.search_range(1,1), obj.search_range(1,2), init_val(:,1));
                    Y = r(obj.search_range(2,1), obj.search_range(2,2), init_val(:,2));
                    Z = r(obj.search_range(3,1), obj.search_range(3,2), init_val(:,3));

                    goal = [X,Y,Z];


            end
        end


        function qTrajectory = jointConfigFromCartesian(obj, inputObj, waypoints, qIn)
            %JOINTCONFIGFROMCARTESIAN Use either pose or 3D XYZ and find
            %the joint configuration for a set of consecutive points in a
            %trajectory
            %[TODO] Check similarity with limb.findIKForGlobalPt

            poseTgt = constraintPoseTarget(inputObj.end_effector);
            poseTgt.ReferenceBody = inputObj.full_tree.BaseName;
            arrayShape = size(waypoints);
            numWaypoints = arrayShape(end); %number of waypoints along the path
            %waypoints can be either [x y z] or a 4x4 Transform
            %Dimensions 3xn or 4x4xn
            dim = length(arrayShape);
            switch dim
                case 3
                    %Then using transforms (pose constraint)
                    tform = waypoints;
                    poseTgt.Weights = [1 0];

                case 2
                    %then using cartesian goals (position)
                    %Convert the waypoints (n x3) in to transforms
                    tform = trvec2tform(waypoints');

                    %Find the dimension of length of the waypoints ( 3 x n)
                    poseTgt.Weights = [0 0.8];



            end

            qTrajectory = zeros([(length(qIn)), numWaypoints]);
            qTrajectory(:,1) = qIn;

            limitJointChange = constraintJointBounds(inputObj.full_tree);
            maxJointChange = obj.maxvelocities.*inputObj.joint_mask*obj.interval;


            for k = 2:numWaypoints
                %get next target waypoint

                poseTgt.TargetTransform = tform(:,:,k);

                initGuess = (qTrajectory(:, k-1) .* inputObj.joint_mask);
                limitJointChange.Bounds = [initGuess - maxJointChange, ...
                    initGuess + maxJointChange];

                [qTrajectory(:,k), solInfo] = obj.gik(qTrajectory(:, k-1), limitJointChange, poseTgt);
                if isnan(qTrajectory(:,k))
                    qTrajectory = qTrajectory(:,1:k-1);
                    warning("IK Produced an invalid pose")
                    break
                end

            end
            qTrajectory(:,1) = [];
        end

        function [obj, memoryCostTime] = updateContactMemory(obj, contact_pointStruct, ~, ~)
            %UPDATECONTACTMEMORY If new contact has been made, update the appropriate memory stores   
            %[COST] Memory time cost of ActionGen class for remembering means
            tStart = tic;
            if strcmp(obj.search_config.MODE{2}, 'GMM')
                tStart = tic;
                obj = obj.updateGMM(contact_pointStruct);
            end
            if strcmp(obj.search_config.MODE{2}, 'mean')
                %Identify which limb to find the joint mask
                %Add pose of antenna at point of contact to the mean
                obj = obj.addPoseToMean(qIn, joint_mask_i);
            end
            if ~isempty(obj.refineSearch)
                tStart = tic;
                obj.refineSearch = obj.refineSearch.setContactMemory(contact_pointStruct);
            end
            memoryCostTime = toc(tStart);

        end

        function obj = updateGM(obj, contact_pointStruct)

            points = cat(1,contact_pointStruct(:).point);
            n = size(points,1);
            d = size(points,2);

            %mu = points;

            I = eye(d,d);
            p = ones([1, n])/n;
            %variance of the distribution around each point
            if strcmp(obj.search_config.SAMPLE.VAR, "IPD")
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
                if ~strcmp(class(obj.search_config.SAMPLE.VAR), "double")
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

            outputTrajectory = obj.jointConfigFromCartesian(neckIn, waypoints, qIn);

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
                    %mandibleOut.trajectory_queue =
                    %obj.jointtrajectory(startQ, endQ, velLimits)
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

