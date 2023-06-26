classdef SampleActionGen
    %% SAMPLEACTIONGEN  Class containing functions that generate joint control trajectories based on cartesian location (rather than desired joint pose as in JointActionGen)

    properties
        interval %Float copied from the Runtime Args set "RATE"
        maxvelocities %10x1 Array of Floats with Maximum joint velocity limits
        gik %A local instance of GeneralizedInverseKinematic solver
        search_config %Copy of the Runtime Args "SEARCH"
        search_range %Copy of the Runtime Args "SEARCH.RANGE"
        memory_length %Copy of the Runtime Args "ANT_MEMORY"
        refineSearch %A sub-class used to implement InformationGain refinement

    end

    methods
        function obj = SampleActionGen(fullTree, RUNTIME_ARGS)
            %SAMPLEACTIONGEN Initialise class
            obj.interval = RUNTIME_ARGS.RATE;
            obj.maxvelocities = RUNTIME_ARGS.SEARCH.VEL;
            obj.search_config = RUNTIME_ARGS.SEARCH;
            obj.search_range = RUNTIME_ARGS.SEARCH.RANGE;

            obj.memory_length = RUNTIME_ARGS.ANT_MEMORY;

            % If the rate of Information Gain (IG) is considered, use it as
            % the refinement class
            if strcmp(RUNTIME_ARGS.SEARCH.REFINE.MODE, 'IG')
                obj.refineSearch = InformationGain(RUNTIME_ARGS);
            else
                obj.refineSearch = struct.empty;
            end

            %Initialise Generalized IK Solver with the full Ant
            %rigidBodyTree
            obj.gik = generalizedInverseKinematics('RigidBodyTree', fullTree);

            %Set the variables to modify the results from IK to consider
            %the joint values and the pose of the end effector
            obj.gik.ConstraintInputs = {"joint", "pose"};

            %Random restart is used to restart the search for an optimal IK
            %solution, when enabled it takes too long to simulate, so set
            %to false
            obj.gik.SolverParameters.AllowRandomRestart = false;

        end



        function [antennaOut, antennaTrajectoryTime] = loadAntennaTrajectory(obj, antennaIn, qIn, positionIn)
            %% LOADANTENNATRAJECTORY Function to re-load the antennal joint trajectory stored in the antennaIn instance of "Limb" based on the control method set for the Trial
            % Input:
            % obj - Current instance of the SampleActionGen clas
            % antennaIn - An instance of the Limb class corresponding to an
            % antenna
            % qIn - 10x1 array of joint values at the point of loading the
            % trajectory
            % positionIn - 1x4 array of the current model global position
            % [x y z yaw]
            % Output:
            % antennaOut - Copy of antennaIn modified to include the new
            % antenna joint trajectory
            % antennaTrajectoryTime - time in seconds to generate a joint
            % trajectory for model cost evaluation purposes
            %%
            %% [COST] Start time to calculate antenna trajectory
            trajStartTime = tic;
            antennaOut = antennaIn;

            %Check whether search refinement is used for this trial
            if ~isempty(obj.refineSearch) &&  ~isempty(obj.refineSearch.cp)
                N = obj.refineSearch.n_sample;
                %Generate N random goals from the search space
                goalArray = obj.randomGoalGen(N);
            else
                %Generate 1 random goal from the search space
                goalArray = obj.randomGoalGen(1);
            end

            %For each cartesian goal, generate the required paths and
            %trajectories to reach that goal
            goalPropStruct = obj.generateGoalProp(antennaIn, qIn, positionIn, goalArray);

            % If refineSearch is enabled, then select the best goal based
            % on refinement criteria
            if ~isempty(obj.refineSearch) && ~isempty(obj.refineSearch.cp)
                goalPropStructOut = obj.refineSearch.refine(goalPropStruct);
            else
                %Select the first goal in the list (index is a remnant of
                %previous indexing requirements)
                goalPropStructOut = goalPropStruct(1);
            end

            % [COST] End the time recording for loadAntennaTrajectory
            antennaTrajectoryTime = toc(trajStartTime);
            % Load the jointPath generated onto the copy of the antenna
            % Limb object
            antennaOut.trajectory_queue = goalPropStructOut.jointPath;

        end

        function [qTrajectory] = jointTrajectory(obj, waypoints, velLimits)
            %% JOINTTRAJECTORY Find intermediate joint positions between the waypoint joint values
            % Input:
            % obj - current instance of the SampleActionGen class
            % waypoints - [n x m] - n joint values for the subtree
            % evaluated across m waypoints
            % velLimits - [n x 1] - n joint velocity limits corresponding to the n joints in the sub-rigidBodyTree
            % Output:
            % qTrajectory - [n x i] - joint trajectory across i steps -
            % length varies based on velocity limits
            %%
            % Initially divide the trajectory into 500 samples
            numSamples = 500;

            %Calculate the joint values q and the corresponding time t that
            %adheres to velocity limits using the trapezoidal velocity
            %limits
            [q, ~, ~, t] = trapveltraj(waypoints,numSamples,...
                PeakVelocity=velLimits);

            % Scale down the joint times to match the sampling rate (vel
            % limits is in rads/s)
            % This makes the joint times into multiples of the sampling
            % rate
            div_t = t / obj.interval;

            % Round the multiples of joint times to their nearest integers
            round_div_t = round(div_t);

            % Downsample the joint times to find the index of each first multiple of the time step
            [~, i_first_unique] = unique(round_div_t, 'stable');

            % Extract the joint values at these downsampled times
            qTrajectory = [q(:,i_first_unique), q(:,end)];


            %qTrajectory(:,1) = [];


        end

        function waypoints = generateWaypoints(obj, start_pt, end_pt, pattern)
            %% GENERATEWAYPOINTS [DEPRECIATED] Generate a set of cartesian points to form a trajectory between two cartesian points based on a path descriptor
            % Input:
            % obj - the current instance of the SampleActionGen class
            % start_pt - 1x3 cartesian point at the start of the trajectory
            % end_pt - a 1x3 cartesian point at the end of the trajectory
            % pattern - a verbal descriptor of the three different path
            % types: "spiral", "straight" or "curve".
            % Output:
            % waypoints - an 3xn array of cartesian points where n is the
            % number of waypoints and each point is positioned equally
            % along the trajectory
            %%

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
            %% STRAIGHTPATH [DEPRECIATED] Generate a set of cartesian points in a straight line between the start_pt and the end_pt
            % 3xn where n is the number of waypoints

            vector = end_pt - start_pt;

            step_size = 0.07;

            waypoints(1,:) = [start_pt(1) : vector(1)*step_size : end_pt(1)];
            waypoints(2,:) = [start_pt(2) : vector(2)*step_size : end_pt(2)];
            waypoints(3,:) = [start_pt(3) : vector(3)*step_size : end_pt(3)];

        end


        function waypoints = curvePath(~, start_pt, end_pt)
            %% CURVEPATH Generate a set of cartesian points in a curved line between the start_pt and the end_pt
            % 3xn where n is the number of waypoints, curve is defined by
            % 1/4 of a sin wave with the length equal to the interpoint
            % distance

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
            %% SPIRALPATH [DEPRECIATED] Generate a set of cartesian points in a spiral line between the start_pt and the end_pt
            % 3xn where n is the number of waypoints, spiral is defined by
            % cos and sin waves in different axes

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
            %% GENERATEGOALPROP Use the control methods for the antennae, and create a struct containing the goal properties
            % Input:
            % obj - current instance of the SampleActionGen class
            % antennaIn - an instance of the Limb class corresponding to
            % one of the antennae
            % qIn - 10x1 array of the current pose of the model joints
            % positionIn - 1x4 array of the current model position [x y z
            % yaw]
            % goalArray - gx3 array of target cartesian points
            % Output:
            % goalTrajProperties - struct array of the following goal
            % properties:
            % Goal Properties:
            % startPt - The cartesian position of the end effector at the
            % start of the trajectory
            % endPt - The cartesian position of the end effector at the
            % end of the trajectory
            % cartesianPath - (Optional) a set of cartesian points that
            % the end effector passes through on the trajectory towards the
            % endPt
            % jointPath - a set of joint values that move the end effector
            % from the startPt to the endPt
            %%

            %Define the goal struct properties
            goalTrajProperties = struct('startPt', [], 'endPt', [], 'cartesianPath', [], 'jointPath', []);

            % Establish the method of generating goal properties,
            % Through performing IK on all cartesian waypoints
            % OR
            % Finding an IK solution for the endPt and interpolating between qIn and the IK solution
            %---
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
                warning('No option selected for traj generation, default to jointInterp')
                trajGenMode = 'jointInterp';
                trajGenFlag = contains(trajOption, trajGenMode);
            end
            trajGenModeIdx = find(trajGenFlag);
            %---
            %Loop through all goals
            nGoal = size(goalArray,1);
            for g = 1:nGoal
                %StartPt equals the last position of the antenna that was
                %collision free
                goalTrajProperties(g).startPt = antennaIn.free_point;
                %EndPt is the goal location
                goalTrajProperties(g).endPt = goalArray(g,:);

                %If generating trajectory through interpolation of joint
                %poses...
                if trajGenModeIdx==1 %jointInterp

                    %Set the waypoints in the global reference frame
                    %waypointsGlobal = [antennaIn.free_point', goalArray(g,:)'];
                    waypointsGlobal = [goalArray(g,:)'];

                    %Find the joint configurations for the waypoints
                    %[jointWaypoints] = antennaIn.findIKforGlobalPt(waypointsGlobal);
                    freeQIn = qIn;
                    freeQIn(antennaIn.joint_mask == 1) = antennaIn.free_pose;
                    [jointConfig] = obj.jointConfigFromCartesian(antennaIn, waypointsGlobal, freeQIn);

                    %Load joint velocity limits
                    velocityLims = obj.maxvelocities;
                    %velocityLims = obj.maxvelocities(antennaIn.joint_mask==1);
                    %limbJointWaypoints = jointWaypoints(antennaIn.joint_mask == 1,:);
                    jointWaypoints = [freeQIn, jointConfig];

                    %Generate the joint waypoints along the path that
                    %fit to the velocity limits
                    goalTrajProperties(g).jointPath = obj.jointTrajectory(jointWaypoints, velocityLims);

                    %Find the number of intermediate poses
                    nPose = size(goalTrajProperties(g).jointPath,2);

                    % If Search refinement is enabled, and there are some
                    % antennal contacts
                    if ~isempty(obj.refineSearch) && ~isempty(obj.refineSearch.cp)
                        %If the IGEF doesn't need the cartesian path of
                        %the motion for evaluating cost, then don't
                        %generate it
                        if obj.refineSearch.information_measures(2)
                            cartesianOut = nan([nPose,3]);

                            for n = 1:nPose
                                % Find the local cartesian positions based
                                % on joint poses (subtree has the
                                % parent-child transform that matches
                                % ant.position)
                                cartesianOut(n,:) = tbox.findFKlocalPosition(antennaIn.full_tree, ...
                                    goalTrajProperties(g).jointPath(:,n), ...
                                    antennaIn.end_effector);
                            end

                            %Save the cartesian path to reach the goal g
                            goalTrajProperties(g).cartesianPath = cartesianOut;
                        end
                    end


                elseif trajGenModeIdx == 2 %cartesianPath
                    %Generate a trajectory made up of cartesian points
                    %and find the joint configurations to meet those
                    %points.

                    %Use a curved trajectory to generate cartesian
                    %waypoints between startPt and endPt
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
            %% RANDOMGOALGEN Generate n cartesian goals based on a uniform or Gaussian Mixture Model probability distribution
            % Input:
            % obj - current instance of the SampleActionGen class
            % nGoal - integer value for the number of goals to generate
            % Output:
            % goal - nGoal x 3 size array containing each cartesian goal
            %%

            % Identify which distribution to sample from
            switch class(obj.search_range)

                case "gmdistribution"
                    % use random function to sample from gmdistribition
                    goal = random(obj.search_range, nGoal);

                case "double" %Double type defines an array of hard limits of a 3D volume to sample from
                    %Define a function to rescale an array of [0,1] values
                    %to be between [a,b]
                    r = @(a, b, set) (a + (b-a).*set);

                    %Generate an array of random values between 0 and 1
                    %size:nGoalx3
                    init_val = rand([nGoal, 3]);

                    %Use the private function r to rescale the random
                    %initial values
                    X = r(obj.search_range(1,1), obj.search_range(1,2), init_val(:,1));
                    Y = r(obj.search_range(2,1), obj.search_range(2,2), init_val(:,2));
                    Z = r(obj.search_range(3,1), obj.search_range(3,2), init_val(:,3));

                    %Combine to create the cartesian goals
                    goal = [X,Y,Z];

            end
        end


        function qTrajectory = jointConfigFromCartesian(obj, inputObj, waypoints, qIn)
            %% JOINTCONFIGFROMCARTESIAN Use either pose or 3D XYZ and find the joint configuration for a set of consecutive points in a trajectory
            % Input:
            % inputObj - an instance of the Limb class
            % waypoints - Cartesian or transform goal positions or poses for the end effector of inputObj
            % qIn - The current pose of the whole tree
            % Output:
            % qTrajectory - 10xn array of joint configurations for the full
            % ant rigidBodyTree, where n is the number of poses
            %%

            % Set the Limb end-effector to be the controlled link by the
            % gik instance
            poseTgt = constraintPoseTarget(inputObj.end_effector);
            % Control the gik of the end-effector in reference to the full
            % ant rigidBodyTree baselink
            poseTgt.ReferenceBody = inputObj.full_tree.BaseName;
            

            %Read the shape of the waypoints to determine whether
            %extracting from cartesian or transform based goals
            arrayShape = size(waypoints);

            %Use the last dimension to find the number of waypoints along the path
            numWaypoints = arrayShape(end);
            %waypoints can be either [x y z] or a 4x4 Transform
            %Dimensions 3xn or 4x4xn
            dim = length(arrayShape);

            %Transforms are concatenated in the 3rd dimension whereas
            %cartesian points are concatenated in the 2nd dimension
            switch dim
                case 3
                    %Then using transforms (pose constraint)
                    tform = waypoints;
                    poseTgt.Weights = [1 0];

                case 2
                    %then using cartesian goals (position)
                    %Convert the waypoints (n x 3) in to transforms
                    tform = trvec2tform(waypoints');

                    %Find the dimension of length of the waypoints (3 x n)
                    poseTgt.Weights = [0 0.8];
            end

            %Create an array of zeros to contain the output trajectory
            qTrajectory = [qIn, zeros([(length(qIn)), numWaypoints])];
            

            % Add the joint limits to the gik
            limitJointChange = constraintJointBounds(inputObj.full_tree);
            
            % Add the maximum velocities to the gik
            maxJointChange = obj.maxvelocities.*inputObj.joint_mask*obj.interval;
            


            for k = 2:numWaypoints+1
                %get next target waypoint
                %poseTgt.TargetTransform = tform(:,:,k);
                poseTgt.TargetTransform = tform(:,:,k-1);
                %Set the initial guess to be the previous pose of only the
                %relevant joints (minimises drift of pose)
                if size(qIn) == size(inputObj.joint_mask)
                    initGuess = (qTrajectory(:, k-1) .* inputObj.joint_mask);
                else
                    initGuess = qTrajectory(:, k-1);
                end
                % Set the limits to be around the current pose based on
                % velocity limits
                limitJointChange.Bounds = [initGuess - maxJointChange, ...
                    initGuess + maxJointChange];

                % use the gik to find the IK solution based on the enforced
                % constraints
                [qTrajectory(:,k), solInfo] = obj.gik(qTrajectory(:, k-1), limitJointChange, poseTgt);

                if isnan(qTrajectory(:,k)) %If the gik finds an invalid solution (due to other limitations like pose), then go no further and use the trajectory generated so far
                    qTrajectory = qTrajectory(:,1:k-1);
                    warning("IK Produced an invalid pose")
                    break
                end

            end
            % Remove the duplicate initial pose from the full trajectory
            qTrajectory(:,1) = [];
        end

        function [obj, memoryCostTime] = updateContactMemory(obj, contact_pointStruct, ~, ~, ~)
            %% UPDATECONTACTMEMORY If new contact has been made, update the appropriate memory stores
            % (~ is used to maintain the same function structure so the
            % same function name is used between SampleActionGen and
            % JointActionGen)
            % Input:
            % obj - current instance of the SampleActionGen
            % contact_pointStruct - struct containing all contact points
            % gathered so far by the ant model - (point, limb, normal)
            % Output:
            % obj - current instance of SampleActionGen including updates
            % to any sample distributions if required
            % memoryCostTime -[COST] Memory time cost of ActionGen class
            % for storing new contact points and evaluating means
            %%
            %Start cost timer
            tStart = tic;
            % If using Gaussian Mixture Models
            if strcmp(obj.search_config.MODE{2}, 'GMM')
                %tStart = tic;
                obj = obj.updateGMM(contact_pointStruct);
            end

            if ~isempty(obj.refineSearch)
                %tStart = tic;
                obj.refineSearch = obj.refineSearch.setContactMemory(contact_pointStruct);
            end
            % Stop the cost timer
            memoryCostTime = toc(tStart);

        end


        function variance = updateVariance(obj, contactPointArray)
            %% UPDATEVARIANCE Generate a scalar value of variance for use in goal sampling
            % Input:
            % obj - current instance of the SampleActionGen class
            % contactPointArray - nx3 array of cartesian contact locations
            % gathered so far by the ant model
            % Output:
            % variance - scalar value of variance about a mean used to
            % generate future search goals according to the variance
            % generation style
            % Options:
            % 'none' - Use the variance set out in the RuntimeArgs in
            % RUNTIME_ARGS.SEARCH.VAR{2}
            % 'varinc' - The variance increases after the first contact
            % proportional to the number of contacts
            % 'vardec' - The variance decreases after the first contact
            % proportional to the number of contacts
            % 'IPD' - variance is 1/mean of all interpoint distances between
            % contact locations [Not tested]
            %%

            % Identify the variance mode in use
            modeString = obj.search_config.VAR{1};
            availableModes = {'none', 'varinc', 'vardec', 'IPD'};
            modeIndex = find(contains(availableModes, modeString));

            % Find the number of contact points collected so far
            nContact = size(contactPointArray, 1);

            variance_scale = 1;
            % Initialise the variance to be the value set in the
            % RuntimeArgs
            variance = obj.search_config.VAR{2};

            % obj.memory_length = the maximum number of contact
            % points that can be stored in the ant model memory

            if modeIndex == 2 %varinc
                % Start only after at least 2 sensed contact points
                if nContact>=2
                    variance_scale = 1 + (nContact / obj.memory_length);
                end
                variance = variance_scale * obj.search_config.VAR{2};
            elseif modeIndex == 3 %vardec
                % Start only after at least 2 sensed contact points
                if nContact>=2
                    variance_scale = max(0.01, 1 - (nContact/obj.memory_length));
                end
                variance = variance_scale * obj.search_config.VAR{2};
            elseif modeIndex == 4 %IPD
                %[TODO] Not tested IPD Code
                if n > 1
                    distanceMAT = pdist2(contactPointArray, contactPointArray);
                    distanceMAT(distanceMAT==0) = nan;
                    meanIPD = mean(distanceMAT, [], "all");

                    variance = 1/meanIPD;
                end
            end
        end


        function obj = updateGMM(obj, contact_pointStruct)
            %% UPDATEGMM Create and add to the obj a gmdistribution based on the collected contact points
            % All collected contact points by the ant model are used as
            % mean values for a gaussian mixture model. The variance is
            % equal for all points.
            % (It is time consuming to create a new distribution every
            % time, but the other option is to iteratively add points to a
            % GMM which appears to take more time)
            % Input:
            % obj - current instance of the SampleActionGen class
            % contact_pointStruct - struct containing all contact points
            % gathered so far by the ant model - (point, limb, normal)
            % Output:
            % obj - An updated instance of the current SampleActionGen
            % class
            %%

            % Extract only the cartesian location points of the contact
            % points
            points = cat(1,contact_pointStruct(:).point);
            % Find the number of points and the dimensions (3 for cartesian
            % points)
            n = size(points,1);
            d = size(points,2);

            % Make a diagonal matrix to contain the covariance
            I = eye(d,d);
            % Make an array to contain the contribution of each point to
            % the GMM
            p = ones([1, n])/n;

            %variance of the distribution around each point
            variance = obj.updateVariance(points);

            % Fill the diagonal I matrix with the variance, and concatenate
            % in the 3rd dimension for each contact point (no covariance)
            sigma = repmat(I*variance, [1 1 n]);

            % Create a Gaussian Mixture distributon with mean about the
            % contact points, variance and weighting p
            gmObj = gmdistribution(points,sigma,p);

            obj.search_range = gmObj;
        end
      
    end
end

