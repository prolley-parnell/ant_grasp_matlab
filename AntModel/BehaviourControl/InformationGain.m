classdef InformationGain
    %INFORMATIONGAIN Used to evaluate a space in which to search and
    %identify what cartesian point is best to search next

    properties (SetAccess = public, GetAccess = public)
        information_measures
        n_sample

        cp
        cn
        sig1
        sig3
        sigalpha
        mu3


    end

    methods
        function obj = InformationGain(RUNTIME_ARGS)
            %INFORMATIONGAIN Construct an instance of this class
            %   Use the runtime args to define which components are
            %   considered in information gain
            obj.information_measures = RUNTIME_ARGS.SEARCH_SPACE.REFINE.ARG;

            %number of samples to evaluate for finding the highest information gain
            obj.n_sample = 4;

            %Parameters set by Active Tactile Exploration Based on
            %Cost-Aware Information Gain Maximisation
            obj.sig1 = 2;
            obj.sig3 = 2;
            obj.sigalpha = 1;
            obj.mu3 = 2;

        end

        function goalOut = refine(obj, goalArray)
            %REFINE apply the method of refinement appropriate to
            %InformationGain to reduce a set of goal samples to a single
            %goal


            nMeasures = length(obj.information_measures);
            if nMeasures < 1
                goalOut = goalArray(1,:);
                warning("No refinement measures defined, selecting goal 1")
                return
            end
            nGoal = size(goalArray, 2);
            for i=1:nGoal

                goalOut = goalArray(1,:);
                return

                %For each goal, get the information gain measure
                %phi1 = obj.calculatePhi1(goalArray(i));
                
            end


        end

        function obj = setContactMemory(obj, contact_pointStruct)

            obj.cp = cat(1,contact_pointStruct(:).point);
            obj.cn = cat(1,contact_pointStruct(:).normal);

        end

        function metricphi1 = calculatePhi1(obj, goalSample)
            nContact = size(obj.cp,2);
            metricphi1 = nan([nContact,1]);
            for i = 1:nContact
                exp_input = -norm(goalSample - obj.cp(i,:))^2;
                metricphi1(i) = 1 - exp(exp_input/obj.sig1^2);

            end


        end
    end
end

