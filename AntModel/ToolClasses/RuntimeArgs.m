classdef RuntimeArgs
    %RUNTIMEARGS Summary of this class goes here
    %   Detailed explanation goes here

    properties

        PLOT

        % Refresh Rate
        % This is the length of time in seconds that the system pauses for between
        % each time step in the simulation. This is real world time and helps to
        % visualise the model when plotting.
        % Set this value to 0 if not plotting to increase the speed of simulation.
        % Default is 0.01
        RATE(1,1) double = 0.01
        N_TRIALS(1,1) double {mustBeInteger} = 1

        TERMPRINT logical
        TRIAL_NAME
        PRINTOUT
        RECORD

        COLLISION_OBJ

        ANT_URDF string {mustBeFile, mustBeText} = './urdf/ant.urdf'

        ANT_POSE
        ANT_POSITION(1,4) double = [0 0 0 0]

        ANTENNA_CONTROL
        ANT_MEMORY(1,1) double = 20

        %SEARCH_SPACE
        SEARCH

        SENSE

        GRASP

        MAP_SIZE
        MAP_POINT_RADIUS

        BODY_NAV_MODE
        BODY_VEL_LIMITS
        BODY_MOTION_ENABLE

    end

    methods
        function obj = RuntimeArgs()
            %RUNTIMEARGS Construct the controlling class (this must be
            %created to run the model)
            %The class definition sets each of the variables to be used in
            %experimental running of the ant model

            %Automatically enable warnings
            warning('on')

            % Individual plots can be enabled/disabled
            % The current plots are
            % Fig 1: RigidBodyTree model of ant and the CollisionBody of the object
            % Fig 2: COM of the CollisionBody, and the contact points in 3D space
            % Default is [1 1] (Both Enabled)
            obj.PLOT.ENABLE = [1 1]; %True if figure should be plotted


            %The range that the simulation plot shows
            %[Xmin Xmax Ymin Ymax Zmin Zmax]
            %Default = [-5 5 -1 8 -1 5]
            obj.PLOT.AXES = [-5 5 -1 8 -1 5];

            %The virtual camera position for watching the ant model
            %[X_elevation, Y_elevation, Z_elevation]
            %Default = [obj.PLOT.AXES([2, 4, 6])] (The limits of the plot
            %axes.
            obj.PLOT.CAMERA = [obj.PLOT.AXES([2, 4, 6])];

            % Refresh Rate
            % This is the length of time in seconds that the system pauses for between
            % each time step in the simulation. This is real world time and helps to
            % visualise the model when plotting.
            % Set this value to 0 if not plotting to increase the speed of simulation.
            % Default is 0.01
            obj.RATE = 0.01;

            %Number of trials
            % The number of times the model will be run for, a trial ends when a grasp
            % location has been decided on for the first grasp attempt.
            obj.N_TRIALS = 1;


            
            % Print output to Terminal
            % Decide whether to print to the terminal the current state of the
            % simulation iteration
            obj.TERMPRINT = false;


            obj.TRIAL_NAME = '';

            % At the end of the simulation, save a CSV with the data collected during
            % each simulation
            obj.PRINTOUT.ENABLE = true;

            %Output file format
            %Matlab uses Tables to process the data and can print it to
            %"text", "spreadsheet" or "xml" (NOTE: XML sometimes causes
            %issues, use "text" for csv format
            obj.PRINTOUT.FORMAT = "spreadsheet";



            % Where to save the printout files at the end of each
            % experiment. The default folder is called "printout_files" and
            % will be created in the same workspace as wherever the
            % AntModelScript.m is called.
            %The folder must already exist 
            obj.PRINTOUT.FOLDER = 'printout';


            %Whether to make a file that has the time stamp, position and
            %pose of the full model to be replayed.
            obj.RECORD.ENABLE = true;


            %The folder name where the recorded data will be printed.
            %type: char array (not string)
            obj.RECORD.FOLDER = 'mat-files';



            obj.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';
            obj.COLLISION_OBJ.POSITION = [0 3 0.5];
            obj.COLLISION_OBJ.ROTATION = [0 0 0]; %roll pitch yaw
            obj.COLLISION_OBJ.SCALE = 0.15;


            %ANT_URDF The file path to the URDF used to define the
            %RigidBodyTree of the ant model
            %obj.ANT_URDF = './urdf/ant.urdf';
            %The specified joint positions of the ant RigidBodyTree at the start of the
            %simulation
            %DEFAULT: The home pose defined by the URDF represented by
            %"home"
            %nx1 array where n is the number of non-fixed joints
            %n = 10
            obj.ANT_POSE = "home";

            %The starting global cartesian position of the ant model
            %DEFAULT: [0 0 0 0]
            %Is in the form [ X, Y, Z, yaw], where yaw is the rotation in
            %radians about the global Z axis.
            obj.ANT_POSITION = [0 0 0 0];

            %The number of contact points stored by the ant. The memory is
            %FIFO, and separate from the print out data and the Centre of
            %Contact calculations.
            %Default = 20; int between 1 and inf
            obj.ANT_MEMORY = 20;



            % Control methods: "goals", "joint" (not yet implemented)
            % Control trajectories: "curve", "spiral", "straight",
            % "joint_traj"


            %%[NEW] More Compact Antenna Control representation
            %obj.SEARCH
            %SEARCH.MODE = {('p2p' or 'joint'), (control mode ('GMM', 'mean')}
            %SEARCH.REFINE = {('IG' or '')}
            %SEARCH.VAR = {('varinc', 'vardec' or 'none'), (variance
            %starting value=0.5)}
            %SEARCH.RANGE = 3x2 matrix containing the cartesian or joint
            %percentage range
            obj = obj.setAntennaControl({});


            %Arguments required for the refinement of sampled points
            %For REFINE, specifically for the Information Gain Estimation
            %Function.
            obj.SEARCH.REFINE.ARG = {'psi1', 'psi2' ,'psi3', 'psi4'};

            obj.SEARCH.REFINE.PARAM = [1.2, 0.5, 1.4, 0.9];


            % Which method is used to evaluate the input sensory data
            % (contact points) to indicate when the ant should grasp
            %options are (string):
            %DEFAULT: "closest_first_pair" : The first pair of contacts that are closer
            %together than the mandible width thresh = 0.1
            %"furthest_first_pair" : The first pair of contacts that are
            %further apart than the threshold distance
            %with the greatest distance thresh = 1 <- if mandible width is
            %considered, the threshold must be less than the mandible width
            %Width of mandibles is usually 0.8626

            %"thresh_COC" : The pair that give the line closest to the
            %Centre of Contact once the threshold has been reached
            %"largest_wrench_vol" : If assessing all contact point pairings
            %with equal forces in the direction of the opposing point being
            %considered, pick the pair that generates the largest wrench
            %space volume

            %New sense.mode method:
            %Which qualities are evaluated of the contact points to
            %synthesise a grasp
            %Cell array containing any of the following: 'dist', 'align'
            %'dist' - Scores the contact points based on their distance
            %'align' - measures the alignment of the grasp forces with the
            %surface normal
            %'axis' - Using a principal axis through the data rather than
            %summarising individual points to generate a grasp
            obj.SENSE.MODE = {'dist','align'};
            obj.SENSE.THRESH = 0;

            %The number of contact points that must be collected before the
            %first goal evaluation is carried out.
            obj.SENSE.MINIMUM_N = 7;

            %Whether to consider grasps that are larger than the reach of
            %the mandibles at the maximum positions
            %true: enables this and ensures that any two points considered for
            %some sensing modes are not wider than the mandible reach
            %false: does not consider this
            obj.SENSE.MAND_MAX = true;

            %Qhull arguments, different modes enabled to get a fast
            %representation of the convex grasp wrench space using QHULL
            obj.GRASP.QUALITY.Q_HULL_ARGS = {'QJ', 'Pp', 'Qt'};
            
            % Force applied by mandibles
            obj.GRASP.FORCE = 1;

            % Friction of the surface of the object for calculations for
            % grasp quality
            obj.GRASP.OBJ_FRICTION = 0.7;


            %Map size (struct)
            %Height and width of the map used for path planning from the
            %current ant Position to the goal position. Used exclusively
            %for path planning. Map should be as large as the search area
            %required to make a valid path to the goal without collision,
            %but larger maps require more time to process when path
            %planning.
            %(the units are the grid squares)
            %Default = 10
            obj.MAP_SIZE.HEIGHT = 10;
            %Default = 10
            obj.MAP_SIZE.WIDTH = 10;

            %The radius of the collision space created around each contact
            %point in the path planning map.
            %NOTE: Previous work decided the radius using the standard
            %deviation of the contact points that occupy the map
            obj.MAP_POINT_RADIUS = 0.05;


            %Method of approaching a goal location
            %NOTE: If using "goal" then the end position location must not
            %be an invalid position on the planning map, or the process
            %will fail
            %string
            %align: when approaching goal, goal is displaced along
            %alignment vector that is 1/4 of the total path length
            %The subsequent approach is along the alignment axis
            %goal: Increment towards the final goal position and
            %orientation without an offset
            obj.BODY_NAV_MODE = "align";

            %Velocity limits of the ant body through space. (full body
            %limit rather than joint limits)
            %Linear velocity Default: 3 units per second
            obj.BODY_VEL_LIMITS.LINEAR = 3;
            %Angular velocity Default: 1.57 radians per second
            obj.BODY_VEL_LIMITS.ANGULAR = 1.57;

            %Whether to enable the movement of the body or not, used for
            %simplifying the model and enabling tests that do not require
            %the body to move.
            obj.BODY_MOTION_ENABLE = 1;


        end

        function disableWarnings(~)
            warning('off')
            warning('off','inhull:degeneracy')
        end

        function obj = setAntennaControl(obj, cellInstruct)
        %SETANTENNACONTROL Parse a cell array that contains char arrays for
        %instructions on how to set the antenna control method
            availableMethod = {'p2p', 'joint'};
            p2pControlMethod = {'GMM', 'mean'};
            refineMethod = {'IG'};
            jointControlMethod = {'mean'};
            varianceMethod = {'varinc', 'vardec'};
            
            
            methodFlag = contains(availableMethod, cellInstruct);
            
            
            if or(all(methodFlag), all(~methodFlag))
                %warning("Must set only 1 antenna control method, default to sweep");
                obj.SEARCH.MODE{1} = 'joint';
                methodFlag = [0 1];
                          
            end
            obj.SEARCH.MODE{1} = availableMethod{methodFlag==1};

            %In case no specific method is defined, default to random
            obj.SEARCH.MODE{2} = 'random';

            if methodFlag(1)
                obj.SEARCH.RANGE = [-2, 2; ...
                                    2, 4; ...
                                    0, 1];
                refineFlag = contains(refineMethod, cellInstruct);
                if any(refineFlag)
                    obj.SEARCH.REFINE.MODE = refineMethod{refineFlag==1};
                else
                    obj.SEARCH.REFINE.MODE = '';
                end
                
                p2pmethodFlag = contains(p2pControlMethod, cellInstruct);
                if any(p2pmethodFlag)
                    obj.SEARCH.MODE{2} = p2pControlMethod{p2pmethodFlag==1};
                end


            elseif methodFlag(2)
                obj.SEARCH.RANGE = [0 0.6;...
                                    0.15 0.8;...
                                    0.9 0.45];
                obj.SEARCH.REFINE = '';
                jointControlFlag = contains(jointControlMethod, cellInstruct);
                if any(jointControlFlag)
                    obj.SEARCH.MODE{2} = jointControlMethod{jointControlFlag == 1};
                end

            end

            varFlag = contains(cellInstruct, 'var');
            %Set the default values in case that the variance is not
            %defined
            obj.SEARCH.VAR = {'none', 0.5};
            if any(varFlag)
                varMethodFlag = contains(varianceMethod, cellInstruct);
                if all(varMethodFlag)
                    obj.SEARCH.VAR{1} = 'var';
                elseif any(varMethodFlag)
                    obj.SEARCH.VAR{1} = varianceMethod{varMethodFlag==1};
                end
                varValueCell = extractAfter(cellInstruct{varFlag==1}, '=');
                varValueInt = str2num(varValueCell);
                if isempty(varValueInt)
                    warning("Var= is used to set the initial int of variance, specify an integer")
                else
                    obj.SEARCH.VAR{2} = varValueInt;
                end
            end

            rangeFlag = contains(cellInstruct, 'range');
            if any(rangeFlag)
                rangeValueStr = extractBetween(cellInstruct{rangeFlag==1}, '[', ']');
                obj.SEARCH.RANGE = str2num(rangeValueStr{:});
            end %Otherwise leave as default


        end

        function obj = initiatePlots(obj)
            if obj.PLOT.ENABLE(1)
                movegui(figure(1), "west")
                axis(obj.PLOT.AXES);
                set(gca, 'CameraPosition', obj.PLOT.CAMERA, 'DataAspectRatioMode', 'manual');
                set(gcf, 'Name', "Ant Model: Top View", 'NumberTitle', 'off')

            end

            if obj.PLOT.ENABLE(2)
                movegui(figure(2), 'east')
                set(gcf, 'Name', "Grasp Data", 'NumberTitle', 'off');
                axis(obj.PLOT.AXES);
                set(gca, 'CameraPosition', obj.PLOT.CAMERA, 'Color', '#c4c4c4', 'Box', 'on', 'GridColor', 'white', 'DataAspectRatioMode', 'manual');

            end
        end

    end
end

