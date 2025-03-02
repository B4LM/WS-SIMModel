classdef Rob < matlab.System & matlab.system.mixin.CustomIcon
    % VISUALIZER2D 2D Robot Visualizer
    %
    % Displays the pose (position and orientation) of an object in a 2D
    % environment. Additionally has the option to display a map as a 
    % robotics.OccupancyGrid or robotics.BinaryOccupancyGrid, object
    % trajectory, waypoints, lidar scans, and/or objects.
    %
    % For more information, see <a href="matlab:edit mrsDocVisualizer2D">the documentation page</a>
    %
    % Copyright 2018-2019 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        robotRadius = 0;    % Robot radius [m]
        mapName = '';       % Map
    end     
    properties(Nontunable, Logical)
        showTrajectory = true;      % Show trajectory
        hasWaypoints = false;       % Accept waypoints
    end

    % Private properties
    properties(Access = private)
        map;                % Occupancy grid
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        TrajHandle;         % Handle to trajectory plot
        trajX = [];         % X Trajectory points
        trajY = [];         % Y Trajectory points
        WaypointHandle;     % Handle to waypoints
        ObjectHandles;      % Handle to objects
    end

    %% METHODS
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)
            % Create figure
            FigureName = 'Robot Visualization';
            FigureTag = 'RobotVisualization';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            obj.ax = axes('parent',obj.fig);   
            hold(obj.ax,'on');
            
            % Show the map
            obj.map = internal.createMapFromName(obj.mapName);
            if ~isempty(obj.map)
                show(obj.map,'Parent',obj.ax);
            end
            
            % Initialize robot plot
            obj.OrientationHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
            if obj.robotRadius > 0
                % Finite size robot
                [x,y] = internal.circlePoints(0,0,obj.robotRadius,17);
                obj.RobotHandle = plot(obj.ax,x,y,'b','LineWidth',1.5);
            else
                % Point robot
                obj.RobotHandle = plot(obj.ax,0,0,'bo', ...
                    'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
            end
            
            % Initialize trajectory
            if obj.showTrajectory
                obj.TrajHandle = plot(obj.ax,0,0,'b.-');
            end
            
            % Initialize waypoints
            if obj.hasWaypoints
                obj.WaypointHandle = plot(obj.ax,0,0, ...
                   'rx','MarkerSize',10,'LineWidth',2);
            end
            
            % Final setup
            title(obj.ax,'Robot Visualization');
            hold(obj.ax,'off'); 
            axis equal          
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,pose,varargin)          
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Unpack the optional arguments
            idx = 1;
            if obj.hasWaypoints % Waypoints
                waypoints = varargin{idx};
                idx = idx + 1;
            end

            if obj.hasObjDetector % Objects and object detections
                objects = varargin{idx};
            end
                       
            % Update the trajectory
            if obj.showTrajectory
               obj.trajX = [obj.trajX;x];
               obj.trajY = [obj.trajY;y];
               set(obj.TrajHandle,'xdata',obj.trajX,'ydata',obj.trajY);
            end
            
            % Update waypoints
            if obj.hasWaypoints && numel(waypoints) > 1
                set(obj.WaypointHandle,'xdata',waypoints(:,1), ...
                                       'ydata',waypoints(:,2));
            end

            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
            if obj.robotRadius > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,obj.robotRadius,17);
                set(obj.RobotHandle,'xdata',xc,'ydata',yc);
                len = max(lineLength,2*obj.robotRadius); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle,'xdata',x,'ydata',y);
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            end
                  
            
            % Update the figure
            drawnow('limitrate')
            
        end

        % Define total number of inputs for system with optional inputs
        function num = getNumInputsImpl(obj)
            num = 1;
            if obj.hasWaypoints
               num = num + 1; 
            end

        end
        
        % Define input port names
        function [namePose,varargout] = getInputNamesImpl(obj)
            namePose = 'pose';
            idx = 1;
            if obj.hasWaypoints
               varargout{idx} = 'waypoints';
               idx = idx + 1;
            end

        end

        % Define icon for System block
        function icon = getIconImpl(~)
            icon = {'Robot','Visualizer'};
        end
        
    end
        
end
