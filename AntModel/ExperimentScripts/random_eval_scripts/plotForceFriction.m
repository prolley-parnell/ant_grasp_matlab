%% Plot the variation in force against friction of the object for a select grasp

close all;
clear all;
scriptFolder = pwd;
filePath = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts\forceResults.mat';

load(filePath, "graspQarray");
FFArray = [graspQarray(:).force; graspQarray(:).friction]';

forceArray = unique([graspQarray(:).force]);
frictionArray = unique([graspQarray(:).friction]);
Z = nan([length(forceArray),length(frictionArray),2]);
for i = 1:length(forceArray)
    for j = 1:length(frictionArray)
        [a,b] = ismember([forceArray(i), frictionArray(j)], FFArray, 'rows');
        Z(i,j,1) = graspQarray(b).volume;
        Z(i,j,2) = graspQarray(b).epsilon;
    end
end

tiledlayout(1,2)
nexttile
hold on
title('Volume')
xlabel('Friction')
ylabel('Force')
zlabel('Volume')
mesh(frictionArray,forceArray, Z(:,:,1))
hold off
nexttile
hold on
title('Epsilon')
xlabel('Friction')
ylabel('Force')
zlabel('Epsilon')
mesh(frictionArray,forceArray, Z(:,:,2))
%mesh(frictionArray, forceArray, ones([length(forceArray), length(frictionArray)])*10^-9)
hold off