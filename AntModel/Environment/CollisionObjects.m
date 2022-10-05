classdef CollisionObjects
    %COLLISIONOBJECTS
    %Generates and contains the handles to all of the objects within the
    %world
    % REQUIREMENTS: Matlab's Partial Differential Equation Toolbox
    % Changelog - Emily Rolley-Parnell 21/09/2022 - Adding normal vectors
    % to the object handles and a DiscreteGeometry object for each object

    properties
        objectHandles
        discreteGeomHandle
        surfNorm
        DT
        COM
        RUNTIME_ARGS

    end

    methods
        function obj = CollisionObjects(RUNTIME_ARGS)
            %COLLISIONOBJECTS Construct an instance of this class
            %  add all of the desired collision objects then show them
            obj.objectHandles = {};
            obj.discreteGeomHandle = {};
            obj.surfNorm = {};
            obj.DT = {};
            obj.COM = {};
            obj = obj.addStl(RUNTIME_ARGS.COLLISION_OBJ);
            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            obj = obj.plotObjects();
        end

        function obj = plotObjects(obj)
            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)

                for i= 1:length(obj.objectHandles)
                    figure(1)
                    hold on;
                    [~, patch] = show(obj.objectHandles{i});
                    patch.FaceAlpha = 0.2;
                    hold off;
                end
            end
            if obj.RUNTIME_ARGS.PLOT.ENABLE(2)
                for i= 1:length(obj.objectHandles)
                    figure(2)
                    hold on

                    global_COM = tbox.local2global(obj.COM{i}, obj.objectHandles{i}.Pose);
                    plot3(global_COM(1), global_COM(2), global_COM(3), 'x','MarkerSize', 8 , 'Color', 'w', 'LineWidth', 2)

                    hold off
                end

            end
        end



        function obj = addStl(obj, ARGS)
            
            %Import mesh from file
            meshObj = importGeometry(ARGS.FILE_PATH);
            %Scale mesh according to RUNTIME_ARGS
            scaled_mesh = scale(meshObj, ones([1,3])*ARGS.SCALE);



            %Find the centre point offset of the mesh
            centre_pt = mean(scaled_mesh.Vertices,1);
            %Use transforms in case rotation is used too
            t_zero = trvec2tform(-centre_pt);
            %Zero the vertices so they are all about [0 0 0]
            scaled_zero_mesh = translate(scaled_mesh, tform2trvec(t_zero));
            %scaled_zero_mesh = scaled_mesh.Vertices + tform2trvec(t_zero);

            %Add CollisionBody for mesh
            collision_mesh = collisionMesh(scaled_zero_mesh.Vertices);
            collision_mesh.Pose = trvec2tform(ARGS.POSITION);


            obj.objectHandles{end+1} = collision_mesh;

            obj.COM{end+1} = mean(collision_mesh.Vertices, 1);

            %Calculate surface normals with face indices

            %Add DiscreteGeometry object for mesh
            translate(scaled_zero_mesh, ARGS.POSITION);
            obj.discreteGeomHandle{end+1} = scaled_zero_mesh;

            %Calculate the surface normals for each face
            [obj.DT{end+1}, obj.surfNorm{end+1}] = obj.collisionToDelaunay(scaled_zero_mesh);
            


        end

        function [TR, F] = collisionToDelaunay(~, mesh)
            %% Generate Delaunay Triangulation representation for
            % surface normal vectors

            x = mesh.Vertices(:,1);
            y = mesh.Vertices(:,2);
            z = mesh.Vertices(:,3);

            DT = delaunayTriangulation(x,y,z);
            [T,Xb] = freeBoundary(DT);
            TR = triangulation(T,Xb);

%             P = incenter(TR);
            F = faceNormal(TR);
%             trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3), ...
%                 'FaceColor','cyan','FaceAlpha',0.8);
%             axis equal
%             hold on
%             quiver3(P(:,1),P(:,2),P(:,3), ...
%                 F(:,1),F(:,2),F(:,3),0.5,'color','r');

        end

    end
end

