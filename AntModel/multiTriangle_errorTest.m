%%Test Multi triangle issue
close all
clear
savedMatStruct = load('.\double_triangle_12-30.mat');
FET = savedMatStruct.fbTriang;
nearFaces = savedMatStruct.nearV_neighbours

contactPt = savedMatStruct.pointOnObj;


subTri = triangulation(FET.ConnectivityList(nearFaces{:},:), FET.Points)

%Plot the triangulation
hold on
% axis equal
trimesh(subTri, 'FaceAlpha', 0.4)

plot3(contactPt(1), contactPt(2),contactPt(3), 'o','MarkerSize',7, 'Color', 'r')

%%
%find the distance to the nearest vertex
nearV_Cart = FET.Points(savedMatStruct.nearVertID,:)
nearV_dist = vecnorm(nearV_Cart - contactPt, 2, 2)