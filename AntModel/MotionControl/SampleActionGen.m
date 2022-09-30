classdef SampleActionGen
    %SPIRALACTION controls the two antennal angles
    % [TODO] Write the documentation for this
    % %   ChangeLog: 22/08/22 - Emily Rolley-Parnell - Change
    % body2globaltraj class to have premultiplying rotations and remove old
    % code

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
            obj.maxvelocities = ones([10,1]) * deg2rad(100);
            %Make the scape pedicel joint have a higher speed limit
            obj.maxvelocities([7 10]) = deg2rad(180);

            obj.search_config = RUNTIME_ARGS.SEARCH_SPACE;
            obj.search_range = RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.RANGE;

            if strcmp(RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE, 'IG')
                obj.refineSearch = InformationGain(RUNTIME_ARGS);
            else
                obj.refineSearch = struct.empty;
            end

            %Initialise Generalized IK Solver
            obj.gik = generalizedInverseKinematics('RigidBodyTree', fullTree);

            obj.gik.ConstraintInputs = {"joint", "pose"};
            obj.gik.SolverParameters.AllowRandomRestart = false;

        end

        function [antennaOut] = loadAntennaTrajectory(obj, antennaIn, qIn, positionIn)

            %If the control style of the antenna is to reach goal
            %positions
            %Generate the next goal
            antennaOut = antennaIn;
            goal = obj.selectGoal(antennaIn.free_point);
            %goal = obj.randomGoalGen();
            pattern = antennaIn.control_type(~strcmp(antennaIn.control_type, "goals")); %Find the name of the pattern

            switch pattern

                case "joint_traj"
                    %antennaOut = antennaIn;
                    waypointsGlobal = [antennaIn.free_point', goal'];
                    %localModelTF = tbox.modelPosition2GlobalTF(positionIn);
                    %waypointsLocal = tbox.global2local(waypointsGlobal, localModelTF);
                    [jointWaypoints] = antennaIn.findIKforGlobalPt(waypointsGlobal);
                    velocityLims = obj.maxvelocities(antennaIn.joint_mask==1)*obj.interval;
                    antennaOut.trajectory_queue = obj.jointTrajectory(jointWaypoints, velocityLims);


                otherwise

                    waypointsGlobal = obj.generateWaypoints(antennaIn.free_point, goal, pattern);
                    %Transform waypoints to be relative to the full
                    %rigidBodyTree base_link rather than global coordinates
                    %(for speed) antennaIn, points, positionIn, qIn
                    localModelTF = tbox.modelPosition2GlobalTF(positionIn);
                    waypointsLocal = tbox.global2local(waypointsGlobal, localModelTF);
                    antennaOut = obj.trajfromWP(antennaIn, waypointsLocal, qIn);


            end



        end

        function [qTrajectory] = jointTrajectory(~, waypoints, velLimits)
            numSamples = 10;

            [qTrajectory, ~, ~, ~, ~] = trapveltraj(waypoints, numSamples, 'PeakVelocity', velLimits);
            %Remove the duplicate position
            qTrajectory(:,1) = [];


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
        function [goalOut] = selectGoal(obj, currentPosition)

            %Use the refine method to select one
            if ~isempty(obj.refineSearch)
                N = obj.refineSearch.n_sample;
                goalArray = obj.randomGoalGen(N);
                goalOut = obj.refineSearch.refine(goalArray, currentPosition);

            else
                %Generate N random goals from the search space

                goalOut = obj.randomGoalGen(1);



            end
        end

        function [goal] = randomGoalGen(obj, nGoal)
            %Generate an antennal goal position in the world frame

            switch class(obj.search_range)

                case "gmdistribution"
                    goal = random(obj.search_range, nGoal);

                case "double"

                    r = @(a, b, set) (a + (b-a).*set);
                    %mid = mean(space_limits(1,:));

                    init_val = rand([nGoal, 3]);


                    X = r(obj.search_range(1,1), obj.search_range(1,2), init_val(:,1));
                    Y = r(obj.search_range(2,1), obj.search_range(2,2), init_val(:,2));
                    Z = r(obj.search_range(3,1), obj.search_range(3,2), init_val(:,3));



                    goal = [X,Y,Z];


            end
        end






        function outputObj = trajfromWP(obj, inputObj, waypoints, qIn)

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
                    %Find the dimension of length of the waypoints ( 3 x n)

                    poseTgt.Weights = [0 0.8];

                    %Convert the waypoints (n x3) in to transforms

                    tform = trvec2tform(waypoints');

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
            inputObj.trajectory_queue = qTrajectory;

            outputObj = inputObj;
        end

        function obj = updateContactMemory(obj, contact_pointStruct)
            if strcmp(obj.search_config.SAMPLE.MODE, "GM")
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
%         function outputTrajectory = bodyGoal2Traj(obj, headIn, qIn, goalStructIn)
% 
%             [yawOut,global2goalR] = tbox.findGoalrotmat(goalStructIn);
% 
%             % -- Apply the difference in rotation to the current pose of
%             % the head
% 
%             sourcebody = 'base_link';
%             targetbody = headIn.end_effector;
%             currentPoseTF = getTransform(headIn.full_tree, qIn, targetbody, sourcebody);
%             base2EETF = getTransform(headIn.full_tree, homeConfiguration(headIn.full_tree), targetbody, sourcebody);
% 
% 
% 
%             %goalPose_rotm = global2goalR * tform2rotm(base2EETF);
%             goalPose = rotm2tform(global2goalR) * base2EETF;
% 
% 
%             % -- interpolate between the current pose and the rotated pose
%             tSamples = 0:0.05:1;
%             [waypoints,~,~] = transformtraj(currentPoseTF,goalPose,[0 1],tSamples);
% 
% 
% 
%             outputObj = obj.trajfromWP(headIn, waypoints, qIn);
%             outputTrajectory = outputObj.trajectory_queue;
% 
%         end
        function outputTrajectory = loadNeckTrajectory(obj, neckIn, qIn, goalStructIn)

            [yawOut,global2goalR] = tbox.findGoalrotmat(goalStructIn);

            % -- Apply the difference in rotation to the current pose of
            % the head

            sourcebody = neckIn.base_name;
            targetbody = neckIn.end_effector;
            currentPoseTF = getTransform(neckIn.full_tree, qIn, targetbody, sourcebody);
            base2EETF = getTransform(neckIn.full_tree, homeConfiguration(neckIn.full_tree), targetbody, sourcebody);



            %goalPose_rotm = global2goalR * tform2rotm(base2EETF);
            goalPose = rotm2tform(global2goalR) * base2EETF;


            % -- interpolate between the current pose and the rotated pose
            tSamples = 0:0.05:1;
            [waypoints,~,~] = transformtraj(currentPoseTF,goalPose,[0 1],tSamples);



            outputObj = obj.trajfromWP(neckIn, waypoints, qIn);
            outputTrajectory = outputObj.trajectory_queue;

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

