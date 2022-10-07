classdef graspEvaluator
    %GRASPEVALUATOR Summary of this class goes here
    %   Detailed explanation goes here

    properties
        mu
        m %number of vectors to approximate cone
        force_applied
        qhull_arguments %Which specific modifiers are used in the qhull functions
        mandible_max


    end

    methods
        function obj = graspEvaluator(RUNTIME_ARGS)
            %GRASPEVALUATOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.mu = RUNTIME_ARGS.GRASP.OBJ_FRICTION;
            obj.m = 8;
            obj.force_applied = RUNTIME_ARGS.GRASP.FORCE;
            %obj.qhull_arguments = {'QJ', 'Pp', 'Q9', 'QR0', 'n'};
            obj.qhull_arguments = RUNTIME_ARGS.GRASP.QUALITY.Q_HULL_ARGS;

        end

        function qualityObj = evaluateGoal(obj, goalObj, environmentClass)

            %Find the distance between the two points and the true Centre
            %of Mass
            idx = 1;
            objPose = environmentClass.objectHandles{idx}.Pose;
            objCOM = environmentClass.COM{idx};

            globalCOM = tbox.local2global(objCOM, objPose);

            qualityObj = graspQuality();

            A = goalObj.contact_point_array(1,:);
            B = goalObj.contact_point_array(2,:);
            %Calculate the offset from the COM to the grasp contacts axis
            qualityObj.com_offset = obj.axis2COM(A, B, globalCOM);


            %Calculate the volume and epsilon for the given goal grasp
            %point locations
            forces = obj.genOpposeForces(A, B);
            vertices = environmentClass.objectHandles{idx}.Vertices;

            [qualityObj.volume, qualityObj.epsilon] = obj.findWrenchQuality(forces, [A;B], vertices, globalCOM, objCOM);


        end

        function distance = axis2COM(~, pointA, pointB, COM)

            A2COM = COM - pointA;
            AB = pointB - pointA;

            distance = norm(cross(A2COM, AB)) / norm(AB) ;

        end

        function [forces] = genOpposeForces(obj, contactA, contactB)
            %Generate a pair of forces that point in the direction of
            %the other contact point as if pinched in a vice

            force_a = contactB - contactA;
            forceA_norm = force_a/vecnorm(force_a);
            forceB_norm = -forceA_norm;
            forceA = [forceA_norm,obj.force_applied];
            forceB = [forceB_norm,obj.force_applied];
            forces = [forceA;forceB];


        end
        function [wrench] = wrench3d(obj, forces, contacts, globalCOM, lamda)
            %WRENCH3D calculate the 6D wrench space generated by n contacts
            %in the global frame

            num_contacts = length(contacts(:,1));

            %Initialise the torque
            torque = zeros(3, num_contacts*obj.m);
            %initialise the wrench
            wrench = zeros(6, num_contacts*obj.m);

            for i = 1:num_contacts
                cp_idx = (i - 1)*obj.m; %To fit the force vectors within the matrix

                %If mu > 0, represent forces using friction cone, else, a single vector
                cone = obj.coneFromVec(forces(i,:), obj.mu, obj.m);

                %Find the torque
                for j = 1:obj.m
                    torque(:, cp_idx+j) = cross(contacts(i,:)- globalCOM, cone(j,:));
                end

                %Convert in to wrench space and scale using lambda
                wrench(1:3, cp_idx+1:cp_idx+obj.m) = cone';
                wrench(4:6, cp_idx+1:cp_idx+obj.m) = torque(:, cp_idx+1:cp_idx+obj.m).*lamda;
            end

            wrench = wrench';

        end

        function [volume, epsilon] = findWrenchQuality(obj, forces, points, vertices, globalCOM, objCOM)

            %Find max object radius for scaling factor

            r = max(vecnorm(vertices,2, 2));
            lamda = 1/r;

            wrench = obj.wrench3d(forces, points, globalCOM, lamda);


            [chull, volume] = convhulln(wrench,obj.qhull_arguments);
            epsilon = obj.estability(objCOM, wrench, chull);

        end



        function [epsilon] = estability(obj, point, wrenchT,chull)
            %ESTABILITY Find the largest radius circle that fits within the 6D wrench
            %space
            % If point is within hull, find the minimum distance between point and hull
            if length(point) == 3
                %If the point used is the 3D COM, duplicate it in to 6D
                point = [point,point];
            end

            tol = 1.e-13*mean(abs(wrenchT(:)));
            [isInHull, epsilon] = inhull(point,wrenchT,chull, tol);

            % Calculate the epsilon stability

            %             if epsilon<1e-9 %arbitrary threshold
            %                 epsilon = 0; %Wipe out errors in calculations
            %             end
            if ~isInHull
                epsilon = 0; %Wipe out errors in calculations
            end
        end


        function [cone, R] = coneFromVec(~, vector, mu, m)
            %CONEFROMVEC Uses the force vector and generates a force cone based on the
            %coefficient of friction
            %   Plot a circle with radius f*mu
            %   Transpose the circle in the Z axis along the magnitude of the force
            %   vector
            %   Rotate the translated circle about the axes to align [0 0 1] with the
            %   input vector
            %   Return the array of m forces that make up the polyhedral friction cone

            circ = zeros([m, 3]);
            ref_vec = [0 0 vector(4)];

            if mu > 0
                radius = vector(4)*mu; %Cone Radius

                %Circle points
                for j = 1:m
                    alp = 2*pi*j / m;
                    circ(j, 1) = radius * sin(alp);
                    circ(j, 2) = radius * cos(alp);
                    circ(j, 3) = vector(4); %Make height equal to force magnitude
                end


                % Cross product
                A = ref_vec(1:3);
                B = vector(1:3);



                cross_th = asin(sqrt(sum((cross(A,B)).^2)) / (sqrt(sum(A.^2)) * sqrt(sum(B.^2))));

                %Because the starting vector has +ve Z, if the Z of the desired vector
                %is not the same, they must be over 90 degrees apart
                if  sign(B(3)) == -1
                    cross_th = deg2rad(180) - cross_th;
                end
                
                %[ERROR] cross(B,A) = [0 0 0] - Cannot be used in
                %axang2rotm
                %[Fix]
                %If the cross axis is non-existent because the vector is in line with the Z axis
                if all(~logical(cross(B,A)))
                	rotAxis = [1 0 0]; %Give another standard axis of rotation (perpendicular to Z axis)
                else
                	rotAxis = cross(B,A);
                end

                R = axang2rotm([rotAxis cross_th]);
                cone = circ * R;
                v = sum(cone,1)/8;

                % Remove any rounding errors
                tolflag = abs(v) < 10e-9;
                v(tolflag) = 0;


                % Cross product gives the acute angle but not necessarily the sign, so
                % it could be aligned with the -ve cross product. This fixes this
                if any(sign(v) ~= sign(vector(1:3)))
                    cone = -cone;
                end

            else
                cone = vector(1:3);
            end

        end


    end
end

