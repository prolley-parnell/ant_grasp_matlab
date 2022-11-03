classdef JointActionGen
    %JOINTACTIONGEN Class to generate joint trajectories, not using point
    %to point control, but oscillations to generate antennal sweeping
    %motions
    % %   ChangeLog: 02/11/22 - Emily Rolley-Parnell - Create new class for
    % strictly joint based control

    properties
        interval

        %weights
        maxvelocities
        maxacceleration

        search_range
        search_config
        refineSearch

    end

    methods
        function obj = JointActionGen(fullTree, RUNTIME_ARGS)
            %SAMPLEACTIONGEN A Class to generate the interpolated waypoints
            %for a path of action for an antenna
            obj.interval = RUNTIME_ARGS.RATE;
            obj.maxvelocities = ones([10,1]) * deg2rad(180);
            %Make the scape pedicel joint have a higher speed limit
            obj.maxvelocities([7 10]) = deg2rad(230);

            obj.maxacceleration = [1.5, 0, -1.5];

            obj.search_config = RUNTIME_ARGS.SEARCH_SPACE;
            obj.search_range = RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.RANGE;

            if strcmp(RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE, 'IG')
                obj.refineSearch = InformationGain(RUNTIME_ARGS);
            else
                obj.refineSearch = struct.empty;
            end

        end



        function [antennaOut] = loadAntennaTrajectory(obj, antennaIn, qIn, positionIn)
            %loadAntennaTrajectory
            antennaOut = antennaIn;

            trajectoryQueue = obj.generateJointTrajectory(antennaIn, qIn);

            antennaOut.trajectory_queue = trajectoryQueue;

        end

        function [qTrajectory] = generateJointTrajectory(obj, antennaIn, qIn)
            %generateJointTrajectory given the method of sweeping, generate
            %the next full joint trajectory path for the antenna given the
            %velocity limits for each joint
            homeQ = homeConfiguration(antennaIn.subtree);
            maskedQ = qIn(antennaIn.joint_mask == 1);
            jointIDArray = find(antennaIn.joint_mask == 1);
            %Phase 1 - Prime the antenna to be back near the home position
            qTrajectory_P1 = obj.trapVelGen([maskedQ,homeQ], jointIDArray);

            %Phase 2 - Generate waypoints for the different phases of the
            %sweeping motion
            waypoints = [qTrajectory_P1(:,end), obj.genSweepWP(antennaIn)];
            nPath = size(waypoints,2)-1;
            for i = 1:nPath
                waypoint_i = waypoints(:,i:i+1);
                qTrajectory_P2 = obj.trapVelGen(waypoint_i, jointIDArray);
            end

            %qTrajectory_P2 = obj.trapVelGen([qTrajectory_P1(:,end), waypoints], jointIDArray);

            qTrajectory = [qTrajectory_P1, qTrajectory_P2];


        end

        function [trajectory, velocity] = trapVelGen(obj, jointWaypoints, jointID)
            %Check whether the start and end position are different
            nJoint = size(jointWaypoints,1);
            roundWP = round(jointWaypoints,5);
            equalFlag = roundWP(:,1) == roundWP(:,2);
            waypoints = jointWaypoints;
            waypoints(equalFlag,:) = []
            if isempty(waypoints)
                trajectory = [];
                velocity = [];
            else
            
            [q, qd, ~, t] = trapveltraj(waypoints,500,...
                PeakVelocity=obj.maxvelocities(jointID(~equalFlag)),...
                AccelTime=obj.interval*5);
            div_t = t / obj.interval;
            round_div_t = round(div_t);
            [~, i_first_unique] = unique(round_div_t, 'stable');
            trajectory = ones([nJoint, length(i_first_unique)+1]).*jointWaypoints(:,end);
            
            trajectory(~equalFlag,:) = [q(:,i_first_unique), q(:,end)];
            
            velocity = zeros(size(trajectory));
            velocity(~equalFlag,:) = [qd(:,i_first_unique), qd(:,end)];
            end
        end

        function [waypoints] = genSweepWP(obj, antennaIn)
            activeRange = [0.2 0.8;...
                            0.1 0.8;...
                            0.1 0.9];
            limitRange = antennaIn.joint_limits(:, 2) - antennaIn.joint_limits(:, 1);

            restrictJointLimit = activeRange.*limitRange + antennaIn.joint_limits(:, 1);

            a = restrictJointLimit(1:2, 1);
            b = restrictJointLimit(1:2, 2);
            r = @(a, b, set) (a + (b-a).*set);
            init_val = rand([2,1]);
            jointBase = r(a,b,init_val);
            
            joint3lims = restrictJointLimit(3,:);

            outPath = [jointBase;joint3lims(2)];
            sweepIn = [jointBase;joint3lims(1)];

            waypoints = [outPath, sweepIn];
            

        end

        function [trajectory, v] = customTrapVel(obj, jointWaypoints, jointID)
            %Looking to find a set of intermediate waypoints that match the
            %velocity and accelleration limits but doesn't enforce a time
            %length - assumes using max accelleration - for a single joint and
            %two way points

            velocityLim = obj.maxvelocities(jointID);
            diff = abs(jointWaypoints(2) - jointWaypoints(1));
            direction = sign(jointWaypoints(2) - jointWaypoints(1));
            tolerance = diff*0.05;
            approxSampleN = round(diff / (velocityLim*0.5*obj.interval));
            trajectory = nan([approxSampleN,1]);
            trajectory(1) = jointWaypoints(1);
            u = 0; %Initial velocity
            i = 1;
            accelORdecel = 1; %1 or 3 to indicate acceleration or deceleration
            while diff > tolerance
                s = u*obj.interval + 0.5*(obj.interval^2)*obj.maxacceleration(accelORdecel); %displacement
                trajectory(i+1) = trajectory(i) + s*direction;
                diff = abs(trajectory(i+1) - jointWaypoints(2));

                v = min(velocityLim, u + obj.interval*obj.maxacceleration(accelORdecel));

                u = v;
                i = i+1;

            end




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

