classdef Neck
    %Neck Containts the joints and apropriate functions to control the neck
    %joints on the ant model

    properties
        end_effector
        base_name
        joint_mask
        joint_limits
        idx
        subtree
        full_tree
        trajectory_queue
    end

    methods
        function obj = Neck(antTree, RUNTIME_ARGS)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.end_effector = 'mandible_base_link';
            obj.base_name = 'base_link';
            obj.full_tree = antTree;
            obj.joint_mask = [1 1 0 0 0 0 0 0 0 0]';

            obj.trajectory_queue = [];
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
    end
end