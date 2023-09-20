close all;
clear all;

ax = gca;
hold on

plane = collisionBox(1.5,1.5,0.05);
plane.Pose = trvec2tform([0.25 0 -0.025]);
plane.show('Parent', ax);

centerTable = collisionBox(0.5,0.3,0.05);
centerTable.Pose = trvec2tform([0.75 0 0.025]);
[~, patchObj] = centerTable.show('Parent', ax);
patchObj.FaceColor = [0 1 0];

%Add Objects to pick up
leftWidget = collisionCylinder(0.01, 0.07);
leftWidget.Pose = trvec2tform([0.3 -0.65 0.225]);
[~, patchObj] = leftWidget.show('Parent', ax);
patchObj.FaceColor = [1 0 0];

diceWidget = collisionSphere(0.05);
diceWidget.Pose = trvec2tform([0.3 0.35 0.225]);
[~, patchObj] = diceWidget.show('Parent', ax);
patchObj.FaceColor = [0 0.5 0.5];




widgetDimensions = [0.2 0.02 0.07];

stiffness = 1e4;

damping = 30;
transition_region_width = 1e-4;
static_friction_coef = 1;
kinetic_friction_coef = 1;
critical_velocity = 1;



%%
RUNTIME_ARGS = RuntimeArgs();
RUNTIME_ARGS.ANT_URDF = 'ant_description/ant_jaw_only.urdf';
% Object to sense
% Scale is linear scale about the centre of the object, varies for all
% experiments.
RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.04;
% % stl file (binary) to import
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = 'AntModel/Environment/Wedge V1.stl';

RUNTIME_ARGS.COLLISION_OBJ.POSITION = [0, 3, 0];
%Define Ant object
%Import the ant RigidBodyTree from the provided URDF path
robot = importrobot(RUNTIME_ARGS.ANT_URDF);
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.81];
q = homeConfiguration(robot);

configSequence = [robot.Bodies{4}.Joint.PositionLimits; -robot.Bodies{7}.Joint.PositionLimits];
q0 = homeConfiguration(robot); % Position
dq0 = zeros(size(q0)); % Velocity
ddq0 = zeros(size(q0)); % Acceleration


%Define the collision objects in the model environment
env = CollisionObjects(RUNTIME_ARGS);

%open('basic_dynamic_ant.slx')
