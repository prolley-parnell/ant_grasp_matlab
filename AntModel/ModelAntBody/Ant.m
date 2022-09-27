classdef Ant
    %ANT Object class Ant that holds the position and pose properties of an
    %ant RigidBodyTree, as well as its controllers and evaluation classes.
    %   ChangeLog: 22/08/22 - Emily Rolley-Parnell - Updated FreePoint
    %   function to use tbox class

    properties

        antTree
        q
        position

        poseController
        positionController
        graspEval
        senseEval


        limbs
        neckObj

        contact_points
        memory_size


        grasp_complete
        mandible_state



        RUNTIME_ARGS



    end

    methods

        function obj = Ant(RUNTIME_ARGS)
            %ANT Construct an instance of this class
            %Save a copy of the runtime arguments
            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            %Import the ant RigidBodyTree from the provided URDF path
            obj.antTree = importrobot(RUNTIME_ARGS.ANT_URDF);
            obj.antTree.DataFormat = 'column';
            if strcmp(RUNTIME_ARGS.ANT_POSE, "home")
                obj.q = homeConfiguration(obj.antTree);
            elseif isnumeric(RUNTIME_ARGS.ANT_POSE)
                obj.q = RUNTIME_ARGS.ANT_POSE;
            else
                warning("The initial ant model pose is invalid")
            end


            obj.position = RUNTIME_ARGS.ANT_POSITION;



            obj.poseController = PoseControlClass(obj.antTree, RUNTIME_ARGS);
            obj.positionController = PositionControlClass(obj.antTree, RUNTIME_ARGS);


            names = [{"Left_Antenna"}, {"Right_Antenna"}, {"Left_Mandible"}, {"Right_Mandible"}];
            types = [{"Antenna"}, {"Antenna"},{"Mandible"},{"Mandible"}];
            control_type = [{RUNTIME_ARGS.ANTENNA_CONTROL}, {RUNTIME_ARGS.ANTENNA_CONTROL},{nan},{nan}];
            colours = [{'blue'}, {'red'}, {'green'}, {'green'}];
            end_effectors = [{"l_tip"}, {"r_tip"}, {"left_jaw_tip"}, {"right_jaw_tip"}];
            base_names = [{"left_antenna_base"}, {"right_antenna_base"}, {"left_jaw_base"}, {"right_jaw_base"}];

            for i = 1:length(names)
                obj.limbs{i} = Limb(names{i}, types{i}, control_type{i}, colours{i}, end_effectors{i}, base_names{i}, obj.antTree, obj.position, RUNTIME_ARGS);
                obj.limbs{i}.free_point = tbox.findEndEffectorGlobalPosition(obj.antTree, obj.q, obj.position, end_effectors{i});
            end

            obj.neckObj = Neck(obj.antTree,RUNTIME_ARGS);
            obj.contact_points = struct.empty;
            %obj.contact_points = struct("point",[], "limb", []);
            obj.memory_size = RUNTIME_ARGS.ANT_MEMORY;


            obj.senseEval = SenseEvaluator(RUNTIME_ARGS, obj.findMaxMandibleDist);
            obj.graspEval = graspEvaluator(RUNTIME_ARGS);
            obj.mandible_state = 0;
            obj.grasp_complete = 0;

            obj.plotAnt();

        end

        function plotAnt(obj)
            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                figure(1)
                show(obj.antTree, obj.q ,'Parent', gca, 'Position', obj.position, 'PreservePlot',false, 'Collisions', 'off', 'Visual', 'on', 'FastUpdate',true);
            end
        end

        function [obj, sensedData, goal] = update(obj, env)

            %Update global position
            [obj.positionController, obj.position, motionFlag] = obj.positionController.updatePosition(obj.position, obj.q);
            %Plot the ant pose and position
            obj.plotAnt();

            [obj, sensedData] = obj.poseController.updatePose(obj, env, motionFlag);

            %Plot the ant pose and position
            obj.plotAnt();


            %Evaluate the sensory data to instruct behaviour
            [obj, goal] = obj.senseEval.check(obj, sensedData);

            % Update goals
            %If SenseEval indicates to move
            if ~goal.isempty() && ~obj.grasp_complete
                
                goal = goal.setalignment2goal(obj.position);
                goal.plotGoal(obj.RUNTIME_ARGS.PLOT);

                %Evaluate goal
                goal.qualityObj = obj.graspEval.evaluateGoal(goal, env);

                if obj.RUNTIME_ARGS.BODY_MOTION_ENABLE

                    %Add the environmental collision data to the map of the env
                    obj.positionController = obj.positionController.updateGoal(obj.contact_points, obj.position, goal);

                    %Find head pose trajectory
                    obj.neckObj = obj.poseController.newNeckTrajectory(obj.neckObj, obj.q, obj.positionController.goal);
                    % Open Mandibles
                    obj.mandible_state = -1;

                else
                    obj.grasp_complete = 1;
                end


            end
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

        function obj = addContact(obj, contactStruct)
            if ~isempty(contactStruct)
                obj.contact_points(end+1).point = contactStruct.point;
                obj.contact_points(end).normal = contactStruct.normal;
                obj.contact_points(end).limb = contactStruct.limb;

                length_difference = length(obj.contact_points) - obj.memory_size;
                if length_difference > 0
                    obj.contact_points(1:length_difference) = [];
                end
            end
        end

    end


end

