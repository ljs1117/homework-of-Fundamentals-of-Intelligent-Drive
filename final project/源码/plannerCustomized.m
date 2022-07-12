classdef plannerCustomized < nav.algs.internal.InternalAccess

    properties (Access = private)

        InternalMinTurningRadius = 0;
        InternalMotionPrimitiveLength = 0;
        InternalNumMotionPrimitives = 5;
        InternalForwardCost = 1;
        InternalReverseCost = 3;
        InternalDirectionSwitchingCost = 0;
        InternalAnalyticExpansionInterval = 5;
        InternalInterpolationDistance = 1;

    end

    properties (Access = private)

        InternalPathObj;

    end

    properties(Access=private)

        Map;
        Resolution;
        CellSize;
        Dimensions;
        MapExtent;

    end

    properties (Dependent)

        MinTurningRadius;
        MotionPrimitiveLength;
        NumMotionPrimitives;
        ForwardCost;
        ReverseCost;
        DirectionSwitchingCost;
        AnalyticExpansionInterval;
        InterpolationDistance;

    end

    properties

        StateValidator;

    end

    methods

        function obj = plannerCustomized(validator, varargin)

            obj.StateValidator = validator;

            % Getting default names and values
            [defaultNames, defaultValues] = defaultNamesAndValues(obj);

            % Parsing name-value pairs
            parser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            parse(parser, varargin{:});

            % Assigning values to the class properties
            assigningValuesToProperties(obj, parser);

        end

        function [defaultNames, defaultValues] = defaultNamesAndValues(obj)
            %defaultNamesAndValues Returning the properties name and the default
            %   values of the properties

            % Properties of the class
            defaultNames = { 'MotionPrimitiveLength', ...
                'MinTurningRadius', ...
                'NumMotionPrimitives', ...
                'ForwardCost', ...
                'ReverseCost', ...
                'DirectionSwitchingCost', ...
                'AnalyticExpansionInterval', ...
                'InterpolationDistance'};

            % Default value of motion primitive length
            % Setting this value due to the fact that motion primitive
            % should leave the grid cell where the parent node lies
            primitiveLength = ceil(sqrt(2) * obj.CellSize);
            obj.InternalMotionPrimitiveLength = primitiveLength;

            % Default value of minimum turning radius
            % Setting this value due to the reason that motion primitive
            % length cannot exceed one-fourth the length of the
            % circumference of a circle based on the minimum turning radius
            minTurnRad = (2 * primitiveLength) / pi;

            % Making default not to be less than 2
            if minTurnRad < 2

                minTurnRad = 2;

            end

            obj.InternalMinTurningRadius=minTurnRad;

            % Default values of the properties of the class
            defaultValues = { primitiveLength, ...
                minTurnRad, ...
                obj.InternalNumMotionPrimitives, ...
                obj.InternalForwardCost, ...
                obj.InternalReverseCost, ...
                obj.InternalDirectionSwitchingCost, ...
                obj.InternalAnalyticExpansionInterval, ...
                obj.InternalInterpolationDistance};

        end

        function assigningValuesToProperties(obj, parser)

            % Properties on which HLUT calculations depends on
            obj.validateMinimumTurningRadius(parameterValue(parser, 'MinTurningRadius'));
            validateattributes(parameterValue(parser, 'ForwardCost'), {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                'plannerCustomized', 'ForwardCost');
            validateattributes(parameterValue(parser, 'ReverseCost'), {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                'plannerCustomized', 'ReverseCost');

            obj.InternalMinTurningRadius = parameterValue(parser, 'MinTurningRadius');
            obj.InternalForwardCost = parameterValue(parser, 'ForwardCost');
            obj.InternalReverseCost = parameterValue(parser, 'ReverseCost');

            % Properties for which setter needs to be called
            obj.MotionPrimitiveLength = parameterValue(parser, 'MotionPrimitiveLength');
            obj.NumMotionPrimitives = parameterValue(parser, 'NumMotionPrimitives');
            obj.DirectionSwitchingCost = parameterValue(parser, 'DirectionSwitchingCost');
            obj.AnalyticExpansionInterval = parameterValue(parser, 'AnalyticExpansionInterval');
            obj.InterpolationDistance = parameterValue(parser, 'InterpolationDistance');

        end

        function validateMinimumTurningRadius(obj, radius)
            %validateMinimumTurningRadius Validating the length of
            %   minimum turning radius

            % Motion primitive should not have difference of heading
            % greater than pi/2
            errorValue = (2*obj.InternalMotionPrimitiveLength)/pi;
            validateattributes(radius, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', 'positive', '>=', errorValue}, ...
                'plannerCustomized', 'MinTurningRadius');

        end

        function getMapData(obj)
            %getMapData Getting the resolution and dimensions from all types
            %   of supported maps

            % Storing all the supported map internally
            if isa(obj.StateValidator, 'validatorOccupancyMap')

                obj.Map = obj.StateValidator.Map;

                % Converting vehicleCostmap to occupancyMap
            else

                % Extract map resolution
                resolution = 1/obj.StateValidator.Map.CellSize;

                % Create occupancyMap
                obj.Map = occupancyMap(obj.StateValidator.Map.Costmap, resolution);

                % Set properties
                obj.Map.FreeThreshold = obj.StateValidator.Map.FreeThreshold;
                obj.Map.OccupiedThreshold = obj.StateValidator.Map.OccupiedThreshold;
                obj.Map.GridLocationInWorld = obj.StateValidator.Map.MapExtent([1 3]);

            end

            % Extracting data out of the map
            obj.Resolution = obj.Map.Resolution;
            obj.CellSize = 1/obj.Map.Resolution;
            obj.Dimensions = obj.Map.GridSize;
            obj.MapExtent = [obj.Map.XWorldLimits ...
                obj.Map.YWorldLimits];

        end

    end

    % Properties related to nodes and their expansion in continuous space
    properties (Access = private)

        %visitedCellsFront To keep check on the cells which are been
        %   traversed by the tree by forward motion
        visitedCellsFront;

        %visitedCellsBack To keep check on the cells which are been
        %   traversed by the tree by reverse motion
        visitedCellsBack;

        %HeuristicAStarMat Matrix having the length of the path stored
        %   considering the holonomic nature of the vehicle in presence
        %   of the obstacles
        HeuristicAStarMat;

        %AStarPlannerObj Variable to store object to update 2D heuristic
        AStarPlannerObj;

        %PathFound To store the state of the path completion
        PathFound;

        %NumPointsMotionPrimitive Number of points of motion primitives to
        %   be considered according to the step size
        NumPointsMotionPrimitive;

    end

    methods

        function cost = getDubinsHeuristic(obj, start, goal)

            dubConnObj = dubinsConnection("MinTurningRadius", obj.InternalMinTurningRadius);
            [~, cost] = connect(dubConnObj,start,goal);

        end

        function cost = getAStarHeuristic(obj, start, goal)

            startPoint = obj.Map.world2grid(start(1:2));
            goalPoint = obj.Map.world2grid(goal(1:2));

            % Getting index of the points
            index = startPoint(:,1) + size(obj.HeuristicAStarMat,1) * (startPoint(:,2)-1);

            % Getting cost of the index
            cost = obj.HeuristicAStarMat(index);

            if isinf(cost)

                if isempty(obj.AStarPlannerObj.plan(goalPoint, startPoint))

                    return;

                end

                obj.HeuristicAStarMat = min(obj.HeuristicAStarMat, obj.AStarPlannerObj.GCostMatrix(:,:,1));
                cost = obj.HeuristicAStarMat(index);

            end

        end

    end

    methods

        function pathObj = plan(obj, start, goal)

            obj.getMapData();

            obj.StateValidator.StateSpace.StateBounds(1:2,:) = ...
                [obj.MapExtent(1:2); obj.MapExtent(3:4)];

            obj.PathFound = false;

            % Variable to track the node expansion per cell
            obj.visitedCellsFront = false(obj.Dimensions);
            obj.visitedCellsBack = false(obj.Dimensions);

            % Creating object for getting 2D heuristic
            obj.AStarPlannerObj = plannerAStarGrid(obj.Map);

            % Declaring 2D heuristic table
            obj.HeuristicAStarMat = inf * ones(obj.Dimensions);

            % The nodes in priority queue have the following format:
            % [fScore, gScore, hScore, x, y, theta],
            % where fScore is the total score and the priority value.
            % gScore represents the cost from the initial pose, and
            % hScore is the heuristic cost to the goal pose. x, y, theta
            % are the pose of the current node.
            openQueue = nav.algs.internal.PriorityQueue(6, 1);

            closedSet = [];

            % The nodes in the node map have the following format:
            % [x, y, theta]
            nodeMap = nav.algs.internal.NodeMap(3);

            dubConnObj = dubinsConnection("MinTurningRadius", obj.InternalMinTurningRadius);

            gScore = 0;
            hScore = max([obj.getAStarHeuristic(start, goal)...
                obj.getDubinsHeuristic(start, goal)]);

            fScore = gScore + hScore;
            initNode = [fScore, gScore, hScore, start];

            openQueue.push(initNode);
            nodeMap.insertNode(start, 0);

            % 离散化路径曲率
            curvatures = linspace(-1/obj.InternalMinTurningRadius, ...
                1/obj.InternalMinTurningRadius, obj.InternalNumMotionPrimitives);

            % 每隔多长距离检测一次碰撞。
            if isinf(obj.StateValidator.ValidationDistance)
                stepSize = obj.CellSize;
                obj.StateValidator.ValidationDistance = obj.CellSize;
            else
                stepSize = obj.StateValidator.ValidationDistance;
            end

            obj.NumPointsMotionPrimitive = floor(obj.InternalMotionPrimitiveLength/stepSize) + 2;

            % Variables to store number of iterations and nodes
            numIterations = 0;

            % Main loop of Hybrid A*
            while ~(openQueue.isEmpty() || obj.PathFound)

                % Getting the current node and moving it from open list to
                % close list of Hybrid A*
                [currentNode, currentNodeId] = openQueue.top();
                currentPose = currentNode(4:6); % [fScore, gScore, hScore, x, y, theta]
                openQueue.pop();
                numIterations = numIterations + 1;

                if any(ismember(currentPose(1:2),closedSet))
                    continue
                end

                closedSet=[closedSet;obj.Map.world2grid(currentPose(1:2));]; %#ok<AGROW>

                [pathSegObj, ~] = connect(dubConnObj,currentPose,goal);
                length = pathSegObj{1}.Length;
                poses = interpolate(pathSegObj{1},0:stepSize:length);

                if all(obj.StateValidator.isStateValid(poses))

                    obj.PathFound = true;
                    pathPoses = nodeMap.traceBack(currentNodeId);
                    pathPoses = flipud(pathPoses);
                    pathPoses = [start; pathPoses; goal]; %#ok<AGROW>

                    % Convert theta back to degrees
                    pathPoses(:,3) = rad2deg(pathPoses(:,3));

                    % Create a Dubins connection object
                    conn = driving.internal.planning.DubinsConnection.create();
                    conn.MinTurningRadius = obj.InternalMinTurningRadius;

                    % Create an array of Dubins path segments
                    pathSeg = driving.DubinsPathSegment.makeempty();
                    for n = 1 : size(pathPoses,1)-1
                        pathSeg(n) = driving.DubinsPathSegment.create(...
                            conn, pathPoses(n,:), pathPoses(n+1,:));
                    end

                    pathObj = driving.Path.create(pathSeg);

                    break

                end

                for curvature = curvatures

                    deltaTheta = asin(curvature * obj.InternalMotionPrimitiveLength/2);

                    theta = currentPose(3) + deltaTheta;
                    x = currentPose(1) + ...
                        obj.InternalMotionPrimitiveLength.*cos(currentPose(3)+deltaTheta/2);
                    y = currentPose(2) + ...
                        obj.InternalMotionPrimitiveLength.*sin(currentPose(3)+deltaTheta/2);
                    newPose = [x, y, theta];

                    if ~obj.StateValidator.isStateValid(newPose)
                        continue
                    end

                    hScore = max([obj.getAStarHeuristic(newPose, goal)...
                        obj.getDubinsHeuristic(newPose, goal)]);

                    if isinf(hScore)
                        continue
                    end

                    [pathSegObj, pathCosts] = connect(dubConnObj,currentPose,newPose);
                    length = pathSegObj{1}.Length;
                    poses = interpolate(pathSegObj{1},0:stepSize:length);

                    if ~all(obj.StateValidator.isStateValid(poses))
                        continue
                    end

                    gScore = currentNode(2) + pathCosts;

                    if deltaTheta == 0
                        fScore = gScore + hScore;
                    else
                        fScore = gScore + hScore + obj.InternalDirectionSwitchingCost;
                    end

                    openQueue.push([fScore, gScore, hScore, newPose]);
                    nodeMap.insertNode(newPose, currentNodeId);

                end

            end

            obj.InternalPathObj = pathObj;

        end

        function plot(obj)
            [transitionPoses, ~] = interpolate(obj.InternalPathObj);

            plot(obj.StateValidator.Map)
            hold on
            plot(transitionPoses(:,1),transitionPoses(:,2));
            legend off;
            hold off
        end

        function set.StateValidator(obj, validator)
            %set.StateValidator Setter for property StateValidator

            % Validate StateValidator input, all other types are currently not allowed
            validateattributes(validator, {'validatorOccupancyMap', 'validatorVehicleCostmap'}, {}, 'plannerCustomized', 'StateValidator');

            if isa(validator,'validatorVehicleCostmap')

                % Validate validatorVehicleCostmap input
                nav.internal.validation.validateValidatorVehicleCostmap(validator, "plannerCustomized", 'StateValidator');

            else

                % Validate validatorOccupancyMap input
                nav.internal.validation.validateValidatorOccupancyMap(validator, "plannerCustomized", 'StateValidator');

            end

            % Validate stateSpace property of StateValidator input
            coder.internal.errorIf(~ strcmp(validator.StateSpace.Name, 'SE2'), 'nav:navalgs:hybridastar:StateSpaceError');

            obj.StateValidator = validator;
            getMapData(obj);

        end

        function set.MinTurningRadius(obj, radius)
            %set.MinTurningRadius Setter for property Minimum Turning Radius

            validateMinimumTurningRadius(obj, radius);
            obj.InternalMinTurningRadius = radius;

        end

        function MinTurningRadius = get.MinTurningRadius(obj)
            %get.MinTurningRadius Getter for property Minimum Turning Radius

            MinTurningRadius = obj.InternalMinTurningRadius;

        end

        function set.MotionPrimitiveLength(obj, length)
            %set.MotionPrimitiveLength Setter for property length of motion
            %   primitive length

            obj.InternalMotionPrimitiveLength = length;

        end

        function MotionPrimitiveLength = get.MotionPrimitiveLength(obj)
            %get.MotionPrimitiveLength Getter for property length of motion
            %   primitive length

            MotionPrimitiveLength = obj.InternalMotionPrimitiveLength;

        end

        function set.NumMotionPrimitives(obj, numPrimitives)
            %set.NumMotionPrimitives Setter for property number of motion
            %   primitives

            validateattributes(numPrimitives, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'integer', '>=', 3, ...
                'odd'}, 'plannerCustomized', 'NumMotionPrimitives');
            obj.InternalNumMotionPrimitives = numPrimitives;

        end

        function NumMotionPrimitives = get.NumMotionPrimitives(obj)
            %get.NumMotionPrimitives Getter for property number of motion
            %   primitives

            NumMotionPrimitives = obj.InternalNumMotionPrimitives;

        end

        function set.ForwardCost(obj, cost)
            %set.ForwardCost Setter for property forward cost

            validateattributes(cost, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                'plannerCustomized', 'ForwardCost');
            obj.InternalForwardCost = cost;

        end

        function ForwardCost = get.ForwardCost(obj)
            %get.ForwardCost Getter for property forward cost

            ForwardCost = obj.InternalForwardCost;

        end

        function set.ReverseCost(obj, cost)
            %set.ReverseCost Setter for property reverse cost

            validateattributes(cost, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                'plannerCustomized', 'ReverseCost');
            obj.InternalReverseCost = cost;

        end

        function ReverseCost = get.ReverseCost(obj)
            %get.ReverseCost Getter for property reverse cost

            ReverseCost = obj.InternalReverseCost;

        end

        function set.DirectionSwitchingCost(obj, cost)
            %set.DirectionSwitchingCost Setter for property direction switching cost

            validateattributes(cost, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', 'nonnegative'}, ...
                'plannerCustomized', 'DirectionSwitchingCost');
            obj.InternalDirectionSwitchingCost = cost;

        end

        function DirectionSwitchingCost = get.DirectionSwitchingCost(obj)
            %get.DirectionSwitchingCost Getter for property direction switching cost

            DirectionSwitchingCost = obj.InternalDirectionSwitchingCost;

        end

        function set.AnalyticExpansionInterval(obj, numNodes)
            %set.AnalyticExpansionInterval Setter for property analytic
            %   expansion interval

            validateattributes(numNodes, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'integer', 'positive'}, ...
                'plannerCustomized', 'AnalyticExpansionInterval');
            obj.InternalAnalyticExpansionInterval = numNodes;

        end

        function AnalyticExpansionInterval = get.AnalyticExpansionInterval(obj)
            %get.AnalyticExpansionInterval Getter for property analytic
            %   expansion interval

            AnalyticExpansionInterval = obj.InternalAnalyticExpansionInterval;

        end

        function set.InterpolationDistance(obj, stepSize)
            %set.InterpolationDistance Setter for interpolation distance

            validateattributes(stepSize, {'single', 'double'}, ...
                {'nonempty', 'scalar', 'finite', 'real', 'positive'}, ...
                'plannerCustomized', 'InterpolationDistance');
            obj.InternalInterpolationDistance = stepSize;

        end

        function InterpolationDistance = get.InterpolationDistance(obj)
            %get.InterpolationDistance Getter for interpolation distance

            InterpolationDistance = obj.InternalInterpolationDistance;

        end

    end

end