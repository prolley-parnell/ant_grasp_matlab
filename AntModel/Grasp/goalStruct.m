classdef goalStruct
    %goalStruct Properties and functions required for a goalStruct


    properties (SetAccess = public, GetAccess = public)
        qualityObj graspQuality

        time_s = -1
        midpoint = nan([1,3],"double")
        contact_axis = nan([1,3],"double")
        contact_point_array = nan([2,3],"double")
        alignment_axis = nan([1,3],"double")
        goal_z_axis = nan([1,3],"double")
        contact_norm = nan([2,3], "double")


    end
    properties (SetAccess = private, GetAccess = protected)
        emptyFlag = true
    end


    methods
        function obj = goalStruct(~)
            %goalStruct constructor, creates an empty instance of the class
            %quality = A class that can be used and have multiple measures of quality attached

        end

        function [obj, setContactTime] = setcontact(obj, contactStructArray)
            %Validate point size [TODO]
            %% [COST] Start save contacts for goal
            setContactStart = tic;
            %
            pointArray = cat(1,contactStructArray(:).point);
            normArray = cat(1,contactStructArray(:).normal);
            obj.emptyFlag = false;
            obj.contact_point_array = pointArray;
            obj.contact_axis = pointArray(1,:) - pointArray(2,:);
            obj.midpoint = mean(pointArray, 1);
            obj.contact_norm = normArray;
            %
            %[COST] Stop save contacts for goal
            setContactTime = toc(setContactStart);
            
            %[COST] [TODO] Get memory size of goal, either when it is set
            %or as it is maintained in memory
        end

        function flag = isempty(obj)
            flag = obj.emptyFlag;
        end


        function [obj, checkAlignTime] = checkalignment(obj, positionIn)
            %If the distance between the mp+alignment is closer than
            %mp-alignment then invert alignment
            %% [COST] Start Alignment Check time
            checkAlignStart = tic;
            oldAlignment = obj.alignment_axis / norm(obj.alignment_axis);

            oldProjection = obj.midpoint - oldAlignment;
            newProjection = obj.midpoint + oldAlignment;
            oldDist = sqrt(sum((oldProjection - positionIn(1:3)).^2));
            newDist = sqrt(sum((newProjection - positionIn(1:3)).^2));

            if newDist < oldDist
                obj.alignment_axis = -oldAlignment;
            else
                obj.alignment_axis = oldAlignment;
            end

            %
            checkAlignTime = toc(checkAlignStart);
            %[COST] End alignment check time
        end

        function outTable = convert2table(obj)
            time = obj.time_s;
            mp = obj.midpoint;
            axis = obj.contact_axis;
            a = obj.contact_point_array(1,:);
            b = obj.contact_point_array(2,:);
            norm_a = obj.contact_norm(1,:);
            norm_b = obj.contact_norm(2,:);

            goalCell = [{time, mp, axis, a, norm_a, b, norm_b}];
            goalTable = cell2table(goalCell);
            goalTable.Properties.VariableNames = {'Time', 'Midpoint', 'Contact Axis', 'Point A', 'Surf Norm A', 'Point B', 'Surf Norm B'};

            if ~isempty(obj.qualityObj)
                qualityTable = obj.qualityObj.convert2table();
                % append the quality to the table
            else
                qualityTable = [];
            end
            outTable = [goalTable , qualityTable];
        end

        function obj = settime(obj, currentTime)
            obj.time_s = currentTime;
        end

        function [obj, goalCostStruct] = setalignment2goal(obj, globalPosition)
            %SETALIGNMENT2GOAL Find the heading vector for the ant that
            %aligns the body while requiring minimal body movement.
            %% [COST] Start calculating ant head alignment for goal
            setAlignStart = tic;
            
            global_z_vector = [0 0 1];
            obj.alignment_axis = cross(obj.contact_axis, global_z_vector);
            obj = obj.checkalignment(globalPosition);

            ant_z_axis = cross(obj.contact_axis, obj.alignment_axis);
            %Ensure the axis always points up
            obj.goal_z_axis = sign(ant_z_axis(3))*ant_z_axis;
            %
            goalCostStruct.time.align = toc(setAlignStart);
            %[COST] End calculating ant head alignment for goal

           
        end
        
        function obj = plotGoal(obj, PLOT)
            if any(PLOT.ENABLE)
                vector_base = obj.midpoint - obj.alignment_axis;
                nPlot = length(PLOT.ENABLE);
                for i = 1:nPlot
                    if PLOT.ENABLE(i)
                        figure(i)
                        hold on;
                        plot3(obj.contact_point_array(:,1), obj.contact_point_array(:,2), obj.contact_point_array(:,3), ':*','MarkerSize',7, 'LineWidth', 1)
                        quiver3(vector_base(1), vector_base(2), vector_base(3), obj.alignment_axis(1), obj.alignment_axis(2), obj.alignment_axis(3), 'LineWidth', 3, 'Color', 'g')
                        quiver3(obj.midpoint(1), obj.midpoint(2), obj.midpoint(3), obj.goal_z_axis(1), obj.goal_z_axis(2), obj.goal_z_axis(3), 'LineWidth', 3, 'Color', 'b')
                        hold off;
                    end
                end
            end
        end

    end
end