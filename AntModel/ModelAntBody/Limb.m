classdef Limb
    %LIMB Summary of this class goes here
    %   Detailed explanation goes here

    properties
        name
        type
        colour
        end_effector
        base_name
        joint_mask
        joint_limits
        idx
        subtree
        full_tree
        free_point
        free_pose
        trajectory_queue
        collision_latch
        control_type
        motion_state
        
    end

    methods
        function obj = Limb(name, type, control_type, colour, end_effector, base_name, antTree, position, RUNTIME_ARGS)
            %LIMB Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.type = type;
            obj.colour = colour;
            obj.end_effector = end_effector;
            obj.base_name = base_name;
            obj.full_tree = antTree;
            [obj.idx, obj.subtree] = obj.findTreeIdx(antTree, obj.base_name);
            [obj.joint_mask, obj.joint_limits] = obj.findJointMask(antTree, obj.idx);
            obj.trajectory_queue = [];
            obj.collision_latch = 0;
            obj.control_type = control_type;
            obj.motion_state = 0;




            obj = obj.updateTreeTransform(homeConfiguration(antTree), position);



        end



        function obj = updateTreeTransform(obj, qIn, positionIn)
            
            %Find the transform from the base of the full ant to the base
            %of the subtree
            baseTF = getTransform(obj.full_tree, qIn, obj.base_name);
            %Find the transform applied using the [x y z yaw] property of
            %the ant
            % Yaw is CCW rotation but axang takes CW rotation
            positionTF = tbox.modelPosition2GlobalTF(positionIn);
            %Find the combination of origin->full ant base -> subtree base

            globalTF = positionTF * baseTF;

               

            %Set the transform between the parent of the base of the base to be the same as the global
            %transform to compensate for global position change

            setFixedTransform(obj.subtree.Bodies{1,1}.Joint, globalTF);

        end





        function qSub = findMaskSubsetQ(obj, qFull)
            qSub = qFull(obj.joint_mask==1);

        end

        function  qFullMasked = applyMask(obj, qFullAnt, qLocal)
            if size(qLocal,1) ~= size(obj.joint_mask,1)
                masked_goal = zeros(size(obj.joint_mask));
                masked_goal(obj.joint_mask==1) = qLocal;
            else
                masked_goal = obj.joint_mask .* qLocal;
            end
            qFullMasked = qFullAnt.* (1-obj.joint_mask) + masked_goal;

        end

        function bodyTree = makeHeadTree(~, tree)
            head_subtree = copy(tree);
            removeBody(head_subtree, 'left_antenna_base');
            removeBody(head_subtree, 'right_antenna_base');
            bodyTree = head_subtree;

        end

        function subTree = makeSubTree(~, tree, base_name)
            tree_copy = copy(tree);
            if ~strcmp(base_name, tree.BaseName)
                subTree = removeBody(tree_copy, base_name);
            else
                subTree = tree_copy;
            end
        end


        function [body_idx, subtree] = findTreeIdx(obj, tree, varargin)


            validTree = @(x)isa(x, 'rigidBodyTree');
            validText = @(x) isstring(x) || ischar(x) ;


            %If the user provides a subtree, skip ahead
            if and(length(varargin) == 1, validTree(varargin{1}))
                %do nothing new
                subtree = varargin{1};

            elseif validText(varargin{1})
                LR = ['left', 'right'];
                AJ = ['antenna', 'jaw'];
                %If the user provides an effector side as well as a
                %base_link_name, make a new subtree
                if and(any(strcmp(varargin{1}, AJ)) , length(varargin) >1)

                    part_name = AJ(strcmp(varargin{1}, AJ));

                    side_name = LR(strcmp(varargin{2}, LR));

                    %Crop the subtree to only the side specified
                    new_base_name = side_name + '_' + part_name + '_base';
                    subtree = obj.makeSubTree(tree, new_base_name);

                else
                    %If the user provides no subtree but a base_link_name, then
                    %make a new subtree
                    base_link_name = varargin{1};
                    subtree = obj.makeSubTree(tree, base_link_name);
                end

            else
                disp("You must provide reference to a subtree to index")
                return
            end

            link_names = subtree.BodyNames;

            for i = 1:length(link_names)
                name = link_names{i};
                body_idx(i) = find(strcmp(tree.BodyNames, name));

            end
        end

        function [mask, joint_limits] = findJointMask(~, tree, body_idx)
            %Find the mask that represents the joints in the mandibles
            tree_q = [];
            joint_limits = [];

            bodies = tree.Bodies(:);
            for i = 1:length(bodies)

                joint_type = bodies{i}.Joint.Type;

                if ~strcmp(joint_type,'fixed')

                    index = find(body_idx == i);
                    if ~isempty(index)
                        tree_q(end+1) = 1;
                        joint_limits(end+1,:) = bodies{i}.Joint.PositionLimits;

                    else
                        tree_q(end+1) = 0;


                    end
                end
            end

            mask = tree_q';


        end

        function qOut = findIKforGlobalPt(obj, goalPoint)
            nGoal = size(goalPoint,2); %Assumes a 3xn matrix of points
            qOut = [];
            for i = 1:nGoal
                goalPose = trvec2tform(goalPoint(:,i)');
                weights = [0 0 0 0.8 0.8 0.8]; %Weight the tolerance for cartesian position (4:6) rather than orientation (1:3)
                initialguess = homeConfiguration(obj.subtree);
            
                ik = inverseKinematics('RigidBodyTree', obj.subtree);
                [configSol,solInfo] = ik(obj.end_effector,goalPose,weights,initialguess);
                qOut = [qOut, configSol];
                
            end


        end

    end
end

