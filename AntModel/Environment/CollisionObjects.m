classdef CollisionObjects
    %COLLISIONOBJECTS
    %Generates and contains the handles to all of the objects within the
    %world
    % REQUIREMENTS: Matlab's Partial Differential Equation Toolbox
    % Changelog - Emily Rolley-Parnell 31/10/2022 - Adding the capability
    % to import multiple STL

    properties
        objectHandles
        discreteGeomHandle
        surfNorm
        DT
        FBT
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
            obj.FBT = {};
            obj.COM = {};
            obj = obj.addMultiStl(RUNTIME_ARGS.COLLISION_OBJ);
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
            [obj.DT{end+1}, obj.FBT{end+1}] = obj.collisionToDelaunay(scaled_zero_mesh);



        end


        function obj = addMultiStl(obj, ARGS)
            %ADDMULTISTL Add multiple STLs from a folder path
            folderStruct = dir([ARGS.FILE_PATH, '\*.stl']);
            allVerticesArray = [];
            nSTL = length(folderStruct);
            rejectPile = [];
            scaledMeshArray = cell([1,nSTL]);
            for i = 1:nSTL
                    %Import mesh from file
                    meshObj = importGeometry([ARGS.FILE_PATH,'\',folderStruct(i).name]);
                    %Scale mesh according to RUNTIME_ARGS
                    scaledMeshArray{i} = scale(meshObj, ones([1,3])*ARGS.SCALE);
                    if scaledMeshArray{i}.NumFaces < 4
                        rejectPile(end+1) = i;
                    else
                        allVerticesArray = cat(1,allVerticesArray,scaledMeshArray{i}.Vertices);
                    end
            end

            CentreOfMass = mean(allVerticesArray,1);
            
            %Remove any STL files that are too small
            scaledMeshArray(rejectPile) = [];
            
            nSTL_Pruned = length(scaledMeshArray);
            translateMeshArray = cell([1,nSTL_Pruned]);
            for j=1:nSTL_Pruned

                %Translate all vertices so the total mean COM is at 0 0 0
                translateMeshArray{j} = translate(scaledMeshArray{j}, -CentreOfMass);

                %Store the Centre of mass for every STL as the full object COM
                obj.COM{j} = CentreOfMass;

                %Convert the translated meshes to CollisionObjects
                collision_mesh = collisionMesh(translateMeshArray{j}.Vertices);
                collision_mesh.Pose = trvec2tform(ARGS.POSITION);

                obj.objectHandles{j} = collision_mesh;

                %Add DiscreteGeometry object for mesh
                translate(translateMeshArray{j}, ARGS.POSITION);

                %Calculate the surface normals for each face
                [obj.DT{j}, obj.FBT{j}] = obj.collisionToDelaunay(translateMeshArray{j});

            end



            
        end

        function [DT, FBT] = collisionToDelaunay(obj, mesh)
            %% Generate Delaunay Triangulation representation for
            % surface normal vectors

            x = mesh.Vertices(:,1);
            y = mesh.Vertices(:,2);
            z = mesh.Vertices(:,3);

            DT = delaunayTriangulation(x,y,z);
            [T,Xb] = freeBoundary(DT);
            FBT = triangulation(T,Xb);

            %                         %Plotting
            %             P = incenter(FBT);
            %             F = faceNormal(FBT);
            %             trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3), ...
            %                 'FaceColor','cyan','FaceAlpha',0.8);
            %             axis equal
            %             hold on
            %             quiver3(P(:,1),P(:,2),P(:,3), ...
            %                 F(:,1),F(:,2),F(:,3),0.5,'color','r');









        end

        function [triangTetrIdx, triangulationObj, DTsurfaceNormal] = delaunayToTriang(~, delaunayObj)
            %delaunayToTriang - given a tetrahedal connectivity list, and a
            %free boundary from triangulation, find which triangulations
            %match the tetrahedra

            [T,Xb] = freeBoundary(delaunayObj);
            triangulationObj = triangulation(T,Xb);
            triangConnection = triangulationObj.ConnectivityList;
            delaunaytetra = delaunayObj.ConnectivityList;

            % Split the tetrahedrons in to triangles
            tetLabel = [1:size(delaunaytetra,1)]';
            tetMix = [tetLabel, delaunaytetra];
            facets = [tetMix(:,[1 2 3 4]);tetMix(:,[1 2 3 5]);tetMix(:,[1 2 4 5]);tetMix(:,[1 3 4 5])];
            facets(:,2:4) = sort(facets(:,2:4),2);
            facets = sortrows(facets,[2,3,4]);
            duploc = find(all(diff(facets(:,2:4),1) == 0,2));
            facets([duploc;duploc + 1],:) = [];


            %Find the surface normal of the triangulation
            F = faceNormal(triangulationObj);

            %Find the surface normal of the tetrahedra
            %Find which labelled triangle facet matches the DT triang
            %(same numbers, just different order)
            orderTriang = sort(triangConnection,2);
            [~, facetIdx] = ismember(orderTriang, facets(:,2:4), "rows");
            triangTetrIdx = facets(facetIdx,1);

            matchedTriangNorm = [triangTetrIdx, F];
            sortedTriangNorm = sortrows(matchedTriangNorm);

            %Find the average
            DTsurfaceNormal = 1;

            %Plotting
            P = incenter(triangulationObj);
            trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3), ...
                'FaceColor','cyan','FaceAlpha',0.8);
            axis equal
            hold on
            quiver3(P(:,1),P(:,2),P(:,3), ...
                F(:,1),F(:,2),F(:,3),0.5,'color','r');


        end

        function [triangOrder,nnew] = sortFaces(~, triangArray, vertexArray , normalArray)
            triangOrder = triangArray;
            nnew= normalArray*0;
            for j=1:size(triangArray,1)
                v1 = vertexArray(triangArray(j,3),:)- vertexArray(triangArray(j,2),:);
                v2 = vertexArray(triangArray(j,2),:) - vertexArray(triangArray(j,1),:);


                nvek=cross(v2,v1); %calculate normal vectors
                nvek=nvek/norm(nvek);
                nnew(j,:) = nvek;
                dot(nvek, normalArray(j,:))
                if dot(nvek, normalArray(j,:))<0
                    triangOrder(j,:)=[triangArray(j,3), triangArray(j,2), triangArray(j,1)];
                    nnew(j,:)=-nvek;
                end

            end

        end
    end

end

