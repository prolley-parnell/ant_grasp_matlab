classdef Limb
    %% LIMB 
    % A class containing the relevant information for moving extremities on the ant, such as the antennae or mandibles, where each instance represents a single antenna or mandible jaw

    %%

    properties
        name %Identifying name of the limb 
        number %Identifying index of the limb (per type)
        type %Limb type, currently "Antenna" or "Mandible"
        colour %Colour used to plot or visualise contacts made by this limb
        end_effector %String name that is allocated to the rigidBodyTree link at the end of the limb chain
        base_name %String name of the rigidBodyTree link at the base of the kinematic chain making up the limb
        joint_mask %A binary mask where 1 indicates the joints that are relevant to the current Limb of the full rigidBodyTree list of joints
        joint_limits %An array to store the joint maximum and minimum positions, extracted from the rigidBodyTree from the URDF
        idx %Array of integers for the positions of the Limb links in the list of links that make up the full rigidBodyTree
        subtree %a rigidBodyTree that consists only of the kinematic chain for the current Limb
        full_tree %A copy of the full Ant rigidBodyTree
        free_point %A 3x1 array of the last cartesian position of the tip of the limb when the full kinematic chain was not in collision with the environment
        free_pose %A joint position array (with only the joints in this limb) with the last values for when the kinematic chain was not in collision with the environment
        trajectory_queue %An array where each column corresponds to a set of joint positions that make up a full trajectory
        collision_latch %A boolean flag to indicate whether the limb was in collision in the previous time step
        control_type %Taken from the RuntimeArgs.SEARCH.MODE to indicate the control method of antennae
        motion_state %Used mainly with mandibles, -1, 0 or 1 are opening, stationary, closing respectively.
        
    end

    methods
        function obj = Limb(name, number, type, control_type, colour, end_effector, base_name, antTree, position, RUNTIME_ARGS)
            %LIMB Construct an instance of this class
            
            %Copy the constructing variables into the current instance of
            %the class
            obj.name = name;
            obj.number = number;
            obj.type = type;
            obj.colour = colour;
            obj.end_effector = end_effector;
            obj.base_name = base_name;
            obj.full_tree = antTree;
            % Using the full rigidBodyTree, find the sub-rigidBodyTree from
            % the specified starting link to the end of the kinematic
            % chain, as well as the indices of the links that make up the
            % sub-tree
            [obj.idx, obj.subtree] = obj.findTreeIdx(antTree, obj.base_name);
            %Using the sub tree link indices, find the properties of the
            %joints in the kinematic chain
            [obj.joint_mask, obj.joint_limits] = obj.findJointMask(antTree, obj.idx);
            %Initialise variables to be empty, collision free, and
            %stationary
            obj.trajectory_queue = [];
            obj.collision_latch = 0;
            obj.control_type = control_type;
            obj.motion_state = 0;

            % Set the transform for the subtree in this current Limb to
            % match the transform reflected by the full rigidBodyTree in
            % the initial global position
            obj = obj.updateTreeTransform(homeConfiguration(antTree), position);

        end



        function obj = updateTreeTransform(obj, qIn, positionIn)
            %% UPDATETREETRANSFORM Update the transform between the base link of the rigidBodyTree and the base of the kinematic chain to match the position offset of the full Ant
            %Find the transform from the base of the full ant rigidBodyTree to the base
            %of the subtree
            %Input:
            % qIn - The current full joint pose of the Ant rigidBodyTree
            % positionIn - The current position of the Ant in the global
            % frame [x, y, z, yaw] where yaw is the Counter
            % Clockwise rotation about the axis [0 0 1]
            %Output:
            % obj - A copy of the current instance of the Limb class
            %%

            baseTF = getTransform(obj.full_tree, qIn, obj.base_name);
            %Find the position of the ant in transform format
            positionTF = tbox.modelPosition2GlobalTF(positionIn);

            %Find the combined transform from the origin-> full ant base -> subtree base
            globalTF = positionTF * baseTF;

               
            %Set the transform between the parent of the base of the Limb chain 
            % to include the global transform to include full Ant position change
            setFixedTransform(obj.subtree.Bodies{1,1}.Joint, globalTF);

        end





        function qSub = findMaskSubsetQ(obj, qFull)
            %% FINDMASKSUBSETQ Extract the joint values relevant to the current Limb instance
            %Input:
            % obj - The current instance of the Limb class
            % qFull - The 10x1 array of joint positions
            %Output:
            % qSub - an nx1 array where n is the number of variabled joints
            % in the subtree for this Limb
            %%
            qSub = qFull(obj.joint_mask==1);
        end

        function  qFullMasked = applyMask(obj, qFullAnt, qLocal)
            %% APPLYMASK Update only the relevant joint variables to match the desired values and leave the remaining the same
            % Input:
            % obj - The current instance of the Limb class
            % qFullAnt - the previous joint state of the full Ant
            % rigidBodyTree
            % qLocal - either the 10x1 new joint values or nx1 where n is
            % the number of controllable joints for this specific Limb
            % Output:
            % qFullMasked - qFullAnt but with the joints corresponding to
            % those controlled by this specific Limb updated to match those
            % in qLocal
            %%
            %Determine whether the qLocal is 10x1 or nx1
            if size(qLocal,1) ~= size(obj.joint_mask,1)
                %Create a 10x1 array of zeros and fill in the relevant
                %joint values from qLocal
                masked_goal = zeros(size(obj.joint_mask));
                masked_goal(obj.joint_mask==1) = qLocal;
            else
                masked_goal = obj.joint_mask .* qLocal;
            end
            %Modify the joints corresponding to the current limb to match
            %those in the masked goal
            qFullMasked = qFullAnt.* (1-obj.joint_mask) + masked_goal;

        end

        function bodyTree = makeHeadTree(~, tree)
            %% MAKEHEADTREE Remove the antennae from the ant head and return the remaining rigidBodyTree without altering the input tree
            % Input:
            % tree - The full rigidBodyTree including the neck, head,
            % mandibles and antennae
            % Output:
            % bodyTree - The rigidBodyTree that is not linked to the input
            % tree and consists of the neck and head and mandibles.
            %%
            
            %Create a copy of the full rigidBodyTree as to not make changes
            %to the original
            head_subtree = copy(tree);
            
            %Remove both antennae chains from this head_subtree (returns
            %itself)
            removeBody(head_subtree, 'left_antenna_base');
            removeBody(head_subtree, 'right_antenna_base');
            
            bodyTree = head_subtree;

        end

        function subTree = makeSubTree(~, tree, base_name)
            %% MAKESUBTREE Remove all subsequent links after the link determined by base_name and return the remaining tree
            % Does not modify the input tree and instead uses a copy
            % Input:
            % tree - a rigidBodyTree object
            % base_name - a string with a name that matches one of the
            % links found in the rbt tree
            % Output:
            % subTree - a rigidBodyTree object without any links connected
            % after the base_name link

            tree_copy = copy(tree);
            %Checks in case the base_name matches the base of the tree and
            %causes an error
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

