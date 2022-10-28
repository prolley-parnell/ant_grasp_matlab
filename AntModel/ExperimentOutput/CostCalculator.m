classdef CostCalculator
    %CostCalculator contains the functions and values used to evaluate the
    %costs incurred by the model in a single trial. Specifically for the
    %evaluation at the end, not for internal use.

    properties
        grasp_synth_history
        antenna_sample_history
        grasp_synth_start 
        antenna_sample_start
        
    end

    methods
        function obj = CostCalculator(~)
            %COSTCALCULATOR Construct an instance of this class
            %   Initialise the arrays to be empty
            obj.grasp_synth_history = [];
            obj.antenna_sample_history = [];
        end

        function obj = antenna_sampling_stop(obj, tic_time)
            %ANTENNA_SAMPLING_STOP Store the time at which the sampling
            %stops
            %   Using the stored start time and the passed end time, calculate
            % the difference and store it as a history
            obj.antenna_sampling_history(end) = tic_time - obj.antenna_sample_start;
        end

        function obj = grasp_synth_stop(obj, tic_time)
            %GRASP_SYNTH_STOP Store the time at which the recording stops
            %   Using the stores start time and the passed end time,
            %   calculate the duration between
            obj.grasp_synth_history(end) = tic_time - obj.grasp_synth_start;
        end
    end
end