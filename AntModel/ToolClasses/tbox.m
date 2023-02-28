classdef tbox
    %tbox A class containing functions used in transformations
    %between different reference frames, and common model memory
    %manipulations.
    %   ChangeLog: 12/09/22 - Emily Rolley-Parnell - resolvedissues with
    %   global2rotmat by ensuring that all the angles are calculated
    %   against references consistently

    methods (Static)
        function globalPointArray = local2global(localPointArray, localFrameTF)
            %LOCAL2GLOBAL Transform an array of cartesian 3D points from a
            % local reference frame, defined by localTF, to the global frame.
            %Inputs:
            %   localPointArray - nx3 array of cartesian points in a local
            %   frame.
            %   localFrameTF - 4x4 Heterogeneous Transform from the global frame
            %   to the origin of the reference frame used by the
            %   localPointArray.
            % Outputs:
            %   globalPointArray: nx3 array of cartesian points in the
            %   global reference frame.


            %local to global -> rotate then translate
            globalPointArray = (tform2rotm(localFrameTF) * localPointArray')'...
                + tform2trvec(localFrameTF);

        end

        function localPointArray = global2local(globalPointArray, localFrameTF)
            %LOCAL2GLOBAL Transform an array of cartesian 3D points from a
            % local reference frame, defined by localTF, to the global frame.
            %Inputs:
            %   globalPointArray - nx3 array of cartesian points in the
            %   global reference frame.
            %   localFrameTF - 4x4 Heterogeneous Transform from the global
            %   frame origin to the origin of the new frame of reference that will be
            %   used in the localPointArray
            %
            % Outputs:
            %   localPointArray: nx3 array of cartesian points in a
            %   local reference frame defined by localFrameTF.

            %global to local ->  translate then rotate
            localPointArray = tform2rotm(localFrameTF) * (globalPointArray - tform2trvec(localFrameTF)');

        end

        function globalModelTF = modelPosition2GlobalTF(positionIn)
            %modelPosition2GlobalTF Uses the position required to plot a
            %RigidBodyTree and converts it in to a Homogeneous Transform.
            % Inputs:
            %   positionIn: 1x4 vector representing the position of the ant
            %   in global space. [X, Y, Z, Yaw]. Yaw is the Counter
            %   Clockwise rotation about the axis [0 0 1]. This positionIn
            %   refers to the origin of the base_link of the
            %   RigidBodyTree.
            % Outputs:
            %   globalModelTF: positionIn in the form of a 4x4 homogeneous
            %   transform

            translation = positionIn(1:3);

            %RBT PositionIn.Yaw is CW at the base up along the Z axis
            rotation = axang2rotm([0 0 1 positionIn(4)]);
            globalModelTF = [rotation, translation' ; 0 0 0 1];
        end


        function [outputObject, outputValue, successFlag] = popTrajectory(inputObject)

            outputObject = inputObject;

            try
                outputValue = outputObject.trajectory_queue(:,1);
                outputObject.trajectory_queue(:,1) = [];
                successFlag = 1;
            catch
                outputValue = nan;
                successFlag = 0;
                if ~isempty(inputObject.trajectory_queue)
                    warning("Could Not Pop Trajectory of %s", class(inputObject))
                end

            end

        end

        function goalOut = offsetPositionByTransform(goalIn, tformIn, positionIn)
            %OFFSETPOSITIONBYTRANSFORM Given a goal position to navigate to,
            %change that position so that it is reached by a given
            %rigidBodyTree link rather than the base of the robot
            %Input:
            %  TFORM must be the transform from the base link to the new
            %  reference link
            %  positionIn is the desired [X Y Z CCWYaw] of the overall ant head
            %  alignment


            offset = tform2trvec(tformIn)';
            rotatedOffset = tform2rotm(tbox.modelPosition2GlobalTF(goalIn)) * offset;
            goalOut = goalIn;
            goalOut(1:3) = goalIn(1:3) - rotatedOffset';


        end

        function outputArray = replaceLocalTransform(pointArray, currentLocalTF, desiredLocalTF)

        end

        function globalPosition = findFKglobalPosition(RBTree, qIn, positionIn, endEffectorName)
            %FINDENDEFFECTORGLOBALPOSITION Find the cartesian coordinates of an
            %end effector
            % Given the pose of the model, the position in the global frame of
            % the base link of the rigid body tree, and the name of the end
            % effector.

            base2EETF = getTransform(RBTree, qIn, endEffectorName);

            global2baseTF = tbox.modelPosition2GlobalTF(positionIn);

            localEndPosition = tform2trvec(base2EETF);
            globalPosition = tbox.local2global(localEndPosition, global2baseTF);


        end

        function localPosition = findFKlocalPosition(RBsubTree,qIn,endEffectorName)

            base2EETF = getTransform(RBsubTree, qIn, endEffectorName);
            localPosition = tform2trvec(base2EETF);

        end

        function cwAngle = findGlobalAngleOffset(goalVector, referenceAxis, rotationAxis)
            %FINDGLOBALANGLEOFFSET Find the clockwise angle of rotation about a rotationAxis
            %from a given referenceAxis TO the goalVector
            %ReferenceAxis and rotationAxis must be unit in one of the
            %cartesian frames (X, Y or Z)
            % Rotation is calculated with the reference being 12 on the
            % clock and is calculated from an egocentric (from the origin
            % looking out along the axis of rotation)

            rotationAxis_n = rotationAxis/norm(rotationAxis);

            %Mask the comparison vectors according to the axis of rotation
            referenceAxis(rotationAxis_n == 1) = 0;
            goalVector(rotationAxis_n == 1) = 0;


            %Normalise each vector that has been masked
            referenceAxis_m_n = referenceAxis/norm(referenceAxis);
            goalVector_m_n = goalVector/norm(goalVector);


            %Both vectors must be column vectors
            dotProduct = dot(referenceAxis_m_n', goalVector_m_n');
            acuteAngle = acos(dotProduct);

            %Identify how to process the output angle, given the axis of
            %rotation and the goal

            dimension = find(rotationAxis_n == 1);
            %Using left hand rule, if the rotation is about Y axis, then
            %results change
            invertB = 1;
            if dimension == 1
                idx_a = 3;
                idx_b = 2;
                invertB = -1;

            elseif dimension == 2
                idx_a = 3;
                idx_b = 1;

            elseif dimension == 3
                idx_a = 2;
                idx_b = 1;
                invertB = -1;

            else
                warning("Something is incorrect, please select an axis of rotation that is unit in  X,Y or Z global frame")
                idx_a = 0;
                idx_b = 0;

            end


            if and(sign(goalVector_m_n(idx_a)) ~= sign(referenceAxis_m_n(idx_a)), acuteAngle <= pi/2)
                acuteAngle = pi - acuteAngle;
            end
            cwAngle = acuteAngle * invertB * sign(goalVector_m_n(idx_b));

        end


        function [goalYawAngle, headPoseRMat] = findGoalrotmat(goalStructIn)
            %FINDGOALrotmat Give the Yaw of the goal position and the
            %approximate roll and pitch required to align the head
            goalAlignAxis = goalStructIn.alignment_axis;
            goalZAxis = goalStructIn.goal_z_axis;

            %Yaw
            referenceAxisYaw = [0 1 0];
            rotationAxisYaw = [0 0 1];
            goalYawAngle = tbox.findGlobalAngleOffset(goalAlignAxis, referenceAxisYaw, rotationAxisYaw);

            %Rotate the goalZ axis to convert it in to the frame of the
            %alignment axis
            goalZAxis_align_y = [axang2rotm([rotationAxisYaw, -goalYawAngle]) * goalZAxis']';
            goalZAxis_align_y_n = goalZAxis_align_y / norm(goalZAxis_align_y);

            %Pitch
            referenceAxisPitch = [0 0 1];

            rotationAxisPitch = [1 0 0];
            pitchAngle = tbox.findGlobalAngleOffset(goalZAxis_align_y_n, referenceAxisPitch, rotationAxisPitch);

            goalZAxis_align_yp = [axang2rotm([rotationAxisPitch, -pitchAngle]) * goalZAxis_align_y_n']';
            goalZAxis_align_yp_n = goalZAxis_align_yp / norm(goalZAxis_align_yp);

            %Roll
            referenceAxisRoll = [0 0 1];
            rotationAxisRoll = [0 1 0];

            rollAngle = tbox.findGlobalAngleOffset(goalZAxis_align_yp_n, referenceAxisRoll, rotationAxisRoll);

            rollR = axang2rotm([rotationAxisRoll rollAngle]);
            pitchR = axang2rotm([rotationAxisPitch pitchAngle]);

            headPoseRMat = pitchR * rollR;
            %[TODO] Convert angles to euler Z Y X (default) eul2rotm([X Y Z])


        end
        function [pointOnObj, normalVArray, vertIDArray, faceIDArray] = findPointOnObjNormalID(fbTriang, samplePoint)
            %findNormalCollision for a given contact point on a
            %delaunayTriangulation object, and the object, find the normal
            %to the surface at the point of contact.
            %Input:
            % fbTriang - triangulation object for the item with which the
            % collision occured
            % pointOnObj -  nx3 cartesian where n is number of sample points
            %Output: 
            % pointOnObj - The Cartesian point on the object, different
            % from sample point if rounding leads to slight error
            % normalVArray - The surface normal at the point pointOnObj
            % vertIDArray - the vertext ID in the triangulation of the
            % pointOnObject if the collision occured at a vertex
            % faceIDArray - the face ID in the triangulation object if the
            % collision occured on a face plane


            nPoint = size(samplePoint,1);
            pointOnObj = nan(size(samplePoint));
            normalVArray = nan(size(samplePoint));
            vertIDArray = nan([nPoint,1]);
            faceIDArray = nan([nPoint,1]);

            for n = 1:nPoint
                nearVertID = nearestNeighbor(fbTriang, samplePoint(n,:));
                nearV_cartesian = fbTriang.Points(nearVertID, :);
                nearV_dist = vecnorm(nearV_cartesian - samplePoint(n,:), 2, 2);

                if round(nearV_dist, 8) == 0
                    %The point of contact is on the vertex
                    normalVArray(n,:) = vertexNormal(fbTriang, nearVertID);
                    pointOnObj(n,:) = fbTriang.Points(nearVertID,:);
                    vertIDArray(n,:) = nearVertID;
                else

                    nearV_neighbours = vertexAttachments(fbTriang,nearVertID);
                    repTestPt = repmat(samplePoint(n,:), length(nearV_neighbours{:}'),1);
                    B = cartesianToBarycentric(fbTriang, nearV_neighbours{:}', repTestPt);
                    % resolves issues with values that are 0 being marked at -0
                    % (9 sig.fig.)
                    roundB = round(B, 9);

                    %Convert back to cartesian coordinates
                    CB = barycentricToCartesian(fbTriang, nearV_neighbours{:}', roundB);
                    CBDist = vecnorm(CB - samplePoint(n,:) ,2,2);
                    [zero_id] = find(~round(CBDist, 6)); %Find the points where the BC matches C coords

                    if length(zero_id)>1
                        pointOnObj(n,:) = CB(zero_id(1), :);
                        faceIDArray(n,:) = nearV_neighbours{:}(zero_id(1));
                        multiNorm = faceNormal(fbTriang, nearV_neighbours{:}(zero_id)');
                        normalVArray(n,:) = mean(multiNorm);
                        


                    elseif length(zero_id) == 1
                        pointOnObj(n,:) = CB(zero_id, :);
                        faceIDArray(n,:) = nearV_neighbours{:}(zero_id);
                        normalVArray(n,:) = faceNormal(fbTriang, faceIDArray(n,:));

                    else
                        %length(zero_id) == 0, find the closest
                        [~, nearestID] = min(CBDist);
                        pointOnObj(n,:) = CB(nearestID,:);
                        faceIDArray(n,:) = nearV_neighbours{:}(nearestID);
                        normalVArray(n,:) = faceNormal(fbTriang, faceIDArray(n,:));
                        

                    end

                end


            end

        end


        function alignMeasure = findSurfNormAlign(surfaceNormal, forceVector)
            %findSurfNormAlign, gives the proportion of the normal vector at
            %the point of contact that is contained within the force applied at
            %that contact point, can check for multiple normal/force pairs
            %Input: surfaceNormal nx3 unit vector for the normal at point of
            %contact
            % forceVector nx4 non unit vector with (i,1:3) containing the
            % direction of force applied, and (i,4) containing the magnitude of
            % force
            nForce = size(forceVector,1);
            nNorm = size(surfaceNormal, 1);
            alignMeasure = nan([nNorm,1]);
            if nForce ~= nNorm
                warning('Must provide the same number of normals as force vectors');
                return
            end

            for n = 1 : nNorm
                %ensure both surf norm and force are unit vectors
                forceVec_n = -forceVector(n, 1:3)/norm(forceVector(n, 1:3));
                normVec_n = surfaceNormal(n,:) / norm(surfaceNormal(n,:));
                alignMeasure(n) = dot(forceVec_n, normVec_n) / norm(forceVec_n);

            end
        end

        function plotRange(limit)

            vertices = nan(8,3);
            %Find the vertices from the range
            %x min
            vertices(1:4,1) = limit(1,1);
            vertices(5:8,1) = limit(1,2);
            vertices([1,2,7,8],2) = limit(2,1);
            vertices(3:6,2) = limit(2,2);
            vertices([1,4,5,8],3) = limit(3,1);
            vertices([2,3,6,7],3) = limit(3,2);

            faces = [1,2,3,4;5,6,7,8;1,2,7,8;3,4,5,6;1,4,5,8;2,3,6,7];

            figure(1)
            patch('Faces', faces, 'Vertices', vertices, 'FaceColor', '#94a088', 'FaceAlpha', 0.1)

        end

        function limitedValues = applyUpperandLowerLimit(inputValue, limitMat)
            %APPLYUPPERANDLOWERLIMIT Input is an mx1 mat and limit is an
            %mx2 mat with upper limit in the first column, and lower limit
            %on the right column
            value_upper_cap = min(inputValue, limitMat(:,2));
            limitedValues = max(value_upper_cap, limitMat(:,1));

        end

    end
end