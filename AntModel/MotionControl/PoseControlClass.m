classdef PoseControlClass
    %PoseControlCLASS Class that provides joint positions to the Ant
    %Uses an action generator to generate cartesian trajectories, and makes
    %joint waypoints based on those cartesian trajectories.
    %   ChangeLog: 27/09/22 - Emily Rolley-Parnell - Remove the neck
    %   variables to their own class


    properties
        actionGen
        antTree
        interval
        space_limits

        RUNTIME_ARGS


    end

    methods
        function obj = PoseControlClass(antTree, RUNTIME_ARGS)
            %MOTIONCLASS
            obj.actionGen = SampleActionGen(antTree, RUNTIME_ARGS);

            obj.antTree = antTree;

            obj.RUNTIME_ARGS = RUNTIME_ARGS;
            obj.interval = RUNTIME_ARGS.RATE;


        end

        function [ant, contactStructArray] = updatePose(obj, ant, env, motionFlag)
            %Update neck and head
            %[ant, obj] = obj.moveNeck(ant);
            [ant] = obj.moveNeck(ant);
            [ant, contactStructArray] = obj.updateLimbs(ant, env, motionFlag);
        end



        % --------- Generalised Limb Update Function
        function [ant, contactStructArray] = updateLimbs(obj, ant, env, motionFlag)

            limbs = ant.limbs;
            contactStructArray = [];


            for i=1:length(limbs)


                %Move the object rigidBodyTree base to match the global ant
                %position.
                if motionFlag
                    limbs{i} = limbs{i}.updateTreeTransform(ant.q, ant.position);
                end

                ant.plotAnt()
                %If limb is not in collision, store the free point
                [limbs{i}, dataStruct] = obj.tactileSenseEval(limbs{i}, ant.q, ant.position, env);
                contactStructArray = [contactStructArray;dataStruct];



                switch(limbs{i}.type)
                    case "Antenna"
                        [obj, limbs{i}, qLocal] = obj.moveAntenna(limbs{i}, ant.q, ant.position);

                    case "Mandible"
                        [obj, limbs{i}, qLocal] = obj.moveMandible(limbs{i}, ant.q, ant.mandible_state);
                    case "Leg"
                        warning("Leg control is not yet implemented");

                end

                qFull = ant.q;
                %Apply joint mask
                ant.q = limbs{i}.applyMask(qFull, qLocal);



                if ~isempty(dataStruct)
                    ant = ant.addContact(dataStruct);
                    %Update the search space, if this is enabled in the
                    %runtime args
                    obj.actionGen = obj.actionGen.updateContactMemory(ant.contact_points);

                end
            end
            ant.limbs = limbs;


            ant.poseController = obj;
        end

        function [limbOut, dataStruct] = tactileSenseEval(obj, limbIn, qIn, positionIn, env)
            dataStruct = struct.empty;

            %Check for collisions
            [limbOut, contact_point, surface_normal] = obj.collisionCheck(limbIn, qIn, env);

            %If limb is not in collision, store the free point
            if ~limbOut.collision_latch
                limbOut.free_point = tbox.findFKglobalPosition(obj.antTree, qIn, positionIn, limbOut.end_effector);
            end

            %Print collisions
            obj.showPoints(contact_point, surface_normal, limbOut.colour, 1);

            if ~isnan(contact_point)
                if contains(limbOut.name, "Mandible")
                    disp("Mand Contact")
                end
                dataStruct = struct("point", contact_point, "normal", surface_normal, "limb", limbOut.name);
            end

        end

        % --------- Access functions

        function obj = showPoints(obj, collision_points, surface_normal, colour, figure_n)
            if obj.RUNTIME_ARGS.PLOT.ENABLE(figure_n)
                if ~isnan(collision_points)
                    figure(figure_n)
                    hold on;
                    plot3(collision_points(:,1), collision_points(:,2), collision_points(:,3),'o','MarkerSize',5, 'Color', colour)
                    quiver3(collision_points(:,1),collision_points(:,2),collision_points(:,3), ...
                        surface_normal(:,1),surface_normal(:,2),surface_normal(:,3),'off','color','[0.368 0 0.251]');
                    hold off;
                end
            end
        end

        %------------- Collision Checker

        function [limbOut, collision_points, surface_normals] = collisionCheck(obj, limbIn, q, env)

            collision_points = nan;
            %objID = 1;
            surface_normals = nan([1,3]);
            subpose = q(limbIn.joint_mask == 1);
            subtree = limbIn.subtree;
            limbOut = limbIn;
            [areIntersecting, dist ,witnessPoints] = checkCollision(subtree, subpose, env.objectHandles, 'IgnoreSelfCollision','on', 'Exhaustive', 'off');

            [min_dist, ~] = min(dist);
            [body_i, collision_i] = ind2sub(size(dist), find(dist < 0.08));

            if any(isnan(dist))
                warning("RigidBodyTree %s intersection with CollisionObject", limbIn.name)
                limbOut.collision_latch = 1;

            elseif and(~isempty(body_i), ~limbIn.collision_latch)
                if obj.RUNTIME_ARGS.TERMPRINT
                    disp("collision!")
                end

                limbOut.collision_latch = 1;

                %[body_i, collision_i] = ind2sub(size(dist), ind);
                nCollision = length(collision_i);
                pointOnObj = nan([nCollision, 3]);
                for i = 1:nCollision
                    witness_pts = witnessPoints(3*body_i(i)-2:3*body_i(i), 2*collision_i(i)-1:2*collision_i(i));
                    pointOnObj(i,:) = witness_pts(:,2)';
                    surface_normals(i,:) = tbox.findNormalCollision(env.FBT{collision_i(i)}, pointOnObj(i,:));
                end
                collision_points = pointOnObj;

            elseif min_dist>=0.08
                limbOut.collision_latch = 0;
            end
        end


        % ---------- Antenna functions

        function [obj, antennaOut, qOut] = moveAntenna(obj, antennaIn, qIn, globalPosition)
            qOut = qIn;
            antennaOut = antennaIn;
            successFlag = 0;
            if or(antennaIn.collision_latch, isempty(antennaIn.trajectory_queue))
                %If in collision then reset first
                try
                    antennaOut = obj.actionGen.loadAntennaTrajectory(antennaIn, qIn, globalPosition);
                catch
                    warning("Antenna trajectory could not be loaded")
                end
            end

            [antennaOut, q, successFlag] = tbox.popTrajectory(antennaOut);
            if successFlag
                qOut = q;
            else
                warning("You really shouldn't be here - you reloaded the Antenna path and then popped and it still didn't work")
            end

        end


        % -------------- Mandible Functions
        function [obj, mandibleOut, qOut] = moveMandible(obj, mandibleIn, qIn, antMandibleState)
            qOut = qIn;
            mandibleOut = mandibleIn;

            if obj.RUNTIME_ARGS.BODY_MOTION_ENABLE
                if and(antMandibleState ~= mandibleIn.motion_state, ~mandibleIn.collision_latch)
                    %Make a new trajectory
                    mandibleIn.motion_state = antMandibleState;

                    [mandibleOut, reloadSuccessFlag] = obj.actionGen.loadMandibleTrajectory(mandibleIn, qIn);

                    if ~reloadSuccessFlag
                        return
                    end
                end


                if mandibleOut.motion_state ~= 0
                    [mandibleOut, q, successFlag] = tbox.popTrajectory(mandibleOut);
                    if successFlag
                        qOut = q;
                    else
                        if isempty(mandibleOut.trajectory_queue)
                            mandibleOut.motion_state = 0;
                        else
                            warning("You really shouldn't be here - you reloaded the Mandible path and then popped and it still didn't work")
                        end
                    end
                end
            end
        end

        function [neckOut] = newNeckTrajectory(obj, neckIn, qIn, goalStruct)
            neckOut = neckIn;
            neckOut.trajectory_queue = obj.actionGen.loadNeckTrajectory(neckIn, qIn, goalStruct);

        end
   

        % ------------ Neck Pose update function
        function [antOut] = moveNeck(~, antIn)
            antOut = antIn;
            qIn = antIn.q;
            neckIn = antIn.neckObj;

            [neckOut, qLocal, successFlag] = tbox.popTrajectory(neckIn);
            if successFlag
                antOut.neckObj = neckOut;
                antOut.q = neckOut.applyMask(qIn, qLocal);
            end



        end


    end
end

