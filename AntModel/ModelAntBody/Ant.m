classdef Ant
    %ANT Object class Ant that holds the position and pose properties of an
    %ant RigidBodyTree, as well as its controllers and evaluation classes.

    properties

        antTree %RigidBodyTree object
        q %10x1 array of joint values for all non-fixed joints
        position % [x y z yaw]. The x, y, and z elements specify the position in meters, and yaw specifies the yaw angle in radians.

        poseController %Instance of the poseControl Class
        positionController %Instance of the positionControl Class
        graspEval %Instance of the graspEvaluator Class
        graspGen %Instance of the graspSynthesis Class


        limbs %Cell array of instances of the Limb class
        neckObj %Single instanec of the Neck class

        contact_points %Struct containing details of the collision instances with properties point, normal, limb
        memory_size %An integer limit on the length of the contact_points struct


        grasp_complete %Integer flag indicating whether to exit the main loop
        mandible_state %Flag to indicate whether any mandible is in contact with an object (used when using position control)



        RUNTIME_ARGS %A local copy of the RuntimeArgs used to initialise this class, mainly for development and any important variables should be copied at class initialisation



    end

    methods

        function obj = Ant(RUNTIME_ARGS)
            %ANT Construct an instance of this class
            %Save a copy of the RuntimeArgs (RA)
            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            %Import the ant RigidBodyTree from the provided URDF path
            obj.antTree = importrobot(RUNTIME_ARGS.ANT_URDF);
            obj.antTree.DataFormat = 'column';
            %Set the initial joint position according to the hard coded
            %"home" from the URDF or the pose set in RA
            if strcmp(RUNTIME_ARGS.ANT_POSE, "home")
                obj.q = homeConfiguration(obj.antTree);
            elseif isnumeric(RUNTIME_ARGS.ANT_POSE)
                obj.q = RUNTIME_ARGS.ANT_POSE;
            else
                warning("The initial ant model pose is invalid")
            end

            % Set the initial position of the ant (base link is located in
            % the centre of the neck sphere)
            obj.position = RUNTIME_ARGS.ANT_POSITION;

            %Create first instances of the pose and position controllers
            obj.poseController = PoseControlClass(obj.antTree, RUNTIME_ARGS);
            obj.positionController = PositionControlClass(obj.antTree, RUNTIME_ARGS);


            %Manually allocate the names, indices and other properties of
            %the limbs which include the antennae and mandible halves.
            names = [{"Left_Antenna"}, {"Right_Antenna"}, {"Left_Mandible"}, {"Right_Mandible"}];
            numbers = [{1},{2},{1},{2}];
            types = [{"Antenna"}, {"Antenna"},{"Mandible"},{"Mandible"}];
            control_type = [{RUNTIME_ARGS.SEARCH.MODE}, {RUNTIME_ARGS.SEARCH.MODE},{nan},{nan}];
            colours = [{'blue'}, {'red'}, {'green'}, {'green'}];
            % "end_effectors" are the link names found in the URDF of the
            % last link in the chain
            end_effectors = [{"l_tip"}, {"r_tip"}, {"left_jaw_tip"}, {"right_jaw_tip"}];
            % "base names" are the link names found in the URDF of the first link in the chain
            base_names = [{"left_antenna_base"}, {"right_antenna_base"}, {"left_jaw_base"}, {"right_jaw_base"}];

            for i = 1:length(names)
                %Use the properties set above to create instances of the
                %Limb class
                obj.limbs{i} = Limb(names{i}, numbers{i}, types{i}, control_type{i}, colours{i}, end_effectors{i}, base_names{i}, obj.antTree, obj.position, RUNTIME_ARGS);
                % Save a copy of the location of the end effector at the
                % initial pose, when not in collision
                obj.limbs{i}.free_point = tbox.findFKglobalPosition(obj.antTree, obj.q, obj.position, end_effectors{i});
                % Save a copy of the initial pose, when not in collision
                obj.limbs{i}.free_pose = obj.q(obj.limbs{i}.joint_mask==1);
            end

            %Create an instance of the Neck, for head control
            obj.neckObj = Neck(obj.antTree,RUNTIME_ARGS);

            %Initialise the contact_points struct to be empty
            obj.contact_points = struct.empty;

            %Set the memory limit as defined by the runtime args
            obj.memory_size = RUNTIME_ARGS.ANT_MEMORY;

            %Calculate the maximum mandible aperture
            mandible_max = obj.findMaxMandibleDist;

            %Create an instant of the graspSynthesis class using the previously
            %calculated mandible_max
            obj.graspGen = graspSynthesis(RUNTIME_ARGS, mandible_max, obj.findMandibleDepth);

            %Create an instant of the graspEvaluator class
            obj.graspEval = graspEvaluator(RUNTIME_ARGS, mandible_max);

            %Initialise the state of mandible motion to be 0 and stationary
            obj.mandible_state = 0;

            %Initialise the state of the grasp to be incomplete, and 0
            obj.grasp_complete = 0;

            %Call the function to display the model URDF with all of the
            %obj parameters
            obj.plotAnt();

        end

        function plotAnt(obj)
            %% PLOTANT Visualise the model URDF in Figure 1
            % If this specific plot is enabled then display the ant URDF
            % using the pose and position stored in this instance of the
            % class.
            %%
            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                figure(1)
                show(obj.antTree, obj.q ,'Parent', gca, 'Position', obj.position, 'PreservePlot',false, 'Collisions', 'off', 'Visual', 'on', 'FastUpdate',true);
            end
        end

        function [obj, sensedData, goalOut, costStruct] = update(obj, env)
            %% UPDATE The main function for the Ant Class which is called at every time step.
            % Input:
            % obj - the current instance of the class Ant that calls the
            % function
            % env - an instance of the class CollisionObjects
            % Output:
            % obj - the current instance of the class Ant calling the
            % function, with any modifications made by the function itself
            % sensedData - contains any collision information gained during
            % the phase of motion controlled by both the positionController
            % and poseController
            % goalOut - A struct to contain key information about the grasp
            % target generated by graspGen
            % costStruct - Used to track changes across the experiment and
            % accrued costs in time or motion for a single experiment
            %%
            %Initialise an empty cost table
            costStruct = struct();
            

            %Update global position, and position in both the current Ant
            %instance, and the cost struct
            [obj.positionController, obj.position, motionFlag, costStruct.position] = obj.positionController.updatePosition(obj.position, obj.q);
            
            %Plot the ant pose and position
            obj.plotAnt();

            [obj, sensedData, costStruct.pose] = obj.poseController.updatePose(obj, env, motionFlag);

            %Plot the ant pose and position
            obj.plotAnt();


            %Evaluate the sensory data to instruct behaviour
            [obj, goalOut, costStruct.goal] = obj.graspGen.check(obj, sensedData, env);

            % Update goals
            %If graspGenerator indicates to move
            if ~goalOut.isempty() && ~obj.grasp_complete
               
                goalOut.plotGoal(obj.RUNTIME_ARGS.PLOT);

                %Evaluate goal
                goalOut.qualityObj = obj.graspEval.evaluateGoal(goalOut, env);

                % End the experiment trial loop
                obj.grasp_complete = 1;


            end
            %% [COST] Calculate memory space occupied by contact points
            ant_contact_points = obj.contact_points;
            whos_struct = whos('ant_contact_points');
            costStruct.memory.contact_points = whos_struct.bytes;
        end

        function distance = findMaxMandibleDist(obj)

            end_point = [];
            %Return the maximum joint positions of the mandibles
            for i=1:length(obj.limbs)
                if strcmp(obj.limbs{i}.type, "Mandible")
                    %Find the joint limits
                    max_q = obj.limbs{i}.joint_limits(:,2);

                    %Find the end position of the mandible tip given this
                    %pose
                    pose = obj.q;
                    pose(obj.limbs{i}.joint_mask == 1) = max_q;
                    TF = getTransform(obj.antTree, pose, obj.limbs{i}.end_effector);

                    end_point(end+1,:) = tform2trvec(TF);
                end
            end

            A2B = end_point(1,:) - end_point(2,:);
            distance = sqrt(sum(A2B.^2));



        end

        function distance = findMandibleDepth(obj)

            tip = [];
            base = [];
            %Return the maximum joint positions of the mandibles
            for i=1:length(obj.limbs)
                if strcmp(obj.limbs{i}.type, "Mandible")
                    %Find the joint limits
                    max_q = obj.limbs{i}.joint_limits(:,2);

                    %Find the end position of the mandible tip given this
                    %pose
                    pose = obj.q;
                    pose(obj.limbs{i}.joint_mask == 1) = max_q;
                    tip(end+1,:) = tform2trvec(getTransform(obj.antTree, pose, obj.limbs{i}.end_effector));
                    base(end+1,:) = tform2trvec(getTransform(obj.antTree, pose, obj.limbs{i}.base_name));
                
                end
            end

            tip_midpoint = mean(tip,1);
            base_midpoint = mean(base,1);
            
            distance = vecnorm(tip_midpoint - base_midpoint);


        end

        function obj = addContact(obj, contactStruct)
            for n = 1:length(contactStruct)
                if ~isempty(contactStruct)
                    obj.contact_points(end+1).point = contactStruct{n}.point;
                    obj.contact_points(end).normal = contactStruct{n}.normal;
                    obj.contact_points(end).limb = contactStruct{n}.limb;

                    length_difference = length(obj.contact_points) - obj.memory_size;
                    if length_difference > 0
                        obj.contact_points(1:length_difference) = [];
                    end
                end
            end
        end

    end


end

