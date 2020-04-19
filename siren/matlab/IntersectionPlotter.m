classdef IntersectionPlotter < matlab.System
    %INTERSECTIONPLOTTER Helper class for plotting arbitrary intersections
    %   Matlab block usable in Simulink to plot intersections,
    %   traffic lights with color, queue sizes, and rates.
    %   For future implementations, concrete vehicles may be added.
   
    properties(Access = private)
        Figure;
        AxesHandle;
        TrafficLightGraphicObjects = [];
        LabelGraphicObjects = [];
        N_Signals;
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj,varargin)
            figureName = 'Traffic Light Controlled Intersection';
            obj.Figure = findobj('Type', 'Figure', 'Name', figureName);
            
            if isempty(obj.Figure)
                obj.Figure = figure('Name', figureName);
                obj.Figure.NumberTitle = 'off';
                obj.Figure.MenuBar = 'none';
                obj.Figure.ToolBar = 'none';
            end
            
            clf(obj.Figure);
            obj.AxesHandle = axes(obj.Figure);
            
            hold on;
            
            % Set chart limits
            xlim([-0.4 1.4])
            ylim([0 1])
            axis equal

            % Set current figure handle position.
            set(gcf, 'Position', [1 1 800 600]);
            
            % Draw driving surface
            ise = evalin( 'base', 'exist(''segments'',''var'') == 1' );
            if (ise)
                segments = evalin('base', 'segments');
                for i = 1:size(segments, 2)
                    segment = segments(i);

                    mid_vector = segment.end - segment.start;

                    % Get unit orthonormal
                    ortho = [-mid_vector(2), mid_vector(1)];
                    ortho = ortho / norm(ortho);

                    if (isfield(segment, 'width') && ~isempty(segment.width))
                        width = segment.width;
                    else
                        width = 0.1;
                    end

                    % Background
                    shift = segment.lanes / 2 * width * ortho;
                    p1 = segment.start + shift;
                    p2 = segment.start - shift;
                    p3 = segment.end - shift;
                    p4 = segment.end + shift;

                    bg = polyshape([p1; p2; p3; p4]);
                    pbg = plot(bg);
                    pbg.LineStyle = 'none';
                    pbg.FaceColor = '#808080';
                    pbg.FaceAlpha = 1;

                    % Side lines
                    shift = (segment.lanes / 2 - 0.1) * width * ortho;
                    s1 = segment.start + shift;
                    s2 = segment.end + shift;
                    s3 = segment.start - shift;
                    s4 = segment.end - shift;

                    plot([s1(1) s2(1)], [s1(2) s2(2)], 'Color', 'w', 'LineWidth', 2);
                    plot([s3(1) s4(1)], [s3(2) s4(2)], 'Color', 'w', 'LineWidth', 2);

                    % Middle lines
                    for j = 1:segment.lanes - 1
                        if (isfield(segment, 'double_yellow') && ~isempty(segment.double_yellow) && segment.double_yellow == j)
                            shift1 = (segment.lanes / 2 - j + 0.025) * width * ortho;
                            shift2 = (segment.lanes / 2 - j - 0.025) * width * ortho;

                            l1 = segment.start + shift1;
                            l2 = segment.end + shift1;
                            l3 = segment.start + shift2;
                            l4 = segment.end + shift2;

                            plot([l1(1) l2(1)], [l1(2) l2(2)], 'Color', '#ffff00', 'LineWidth', 2);
                            plot([l3(1) l4(1)], [l3(2) l4(2)], 'Color', '#ffff00', 'LineWidth', 2);
                        else
                            shift = (segment.lanes / 2 - j) * width * ortho;

                            l1 = segment.start + shift;
                            l2 = segment.end + shift;

                            plot([l1(1) l2(1)], [l1(2) l2(2)], 'Color', 'w', 'LineWidth', 2, 'LineStyle', '--');
                        end
                    end                
                end
            end
            
            % Connectors
            ise = evalin( 'base', 'exist(''connectors'',''var'') == 1' );
            if (ise)
                connectors = evalin('base', 'connectors');
                for i = 1:size(connectors, 2)
                    connector = connectors(i);

                    bg = polyshape(connector.coords);
                    pbg = plot(bg);
                    pbg.LineStyle = 'none';
                    pbg.FaceColor = '#808080';
                    pbg.FaceAlpha = 1;
                    
                    if (isfield(connector, 'width') && ~isempty(connector.width))
                        width = connector.width;
                    else
                        width = 0.1;
                    end
                    
                    if (isfield(connector, 'double_yellow') && ~isempty(connector.double_yellow))
                        mid_vector = connector.double_yellow(2, :) - connector.double_yellow(1, :);
                        
                        ortho = [-mid_vector(2), mid_vector(1)];
                        ortho = ortho / norm(ortho);
                        
                        shift = 0.025 * width * ortho;

                        l1 = connector.double_yellow(1, :) + shift;
                        l2 = connector.double_yellow(2, :) + shift;
                        l3 = connector.double_yellow(1, :) - shift;
                        l4 = connector.double_yellow(2, :) - shift;

                        plot([l1(1) l2(1)], [l1(2) l2(2)], 'Color', '#ffff00', 'LineWidth', 2);
                        plot([l3(1) l4(1)], [l3(2) l4(2)], 'Color', '#ffff00', 'LineWidth', 2);
                    end

                    if (connector.sides)

                        for j = 1:2:size(connector.coords, 1) - 1
                            x = connector.coords(j:j+1, 1);
                            y = connector.coords(j:j+1, 2);

                            ortho = [y(1) - y(2), x(2) - x(1)];
                            ortho = ortho / norm(ortho);

                            shift = width * 0.1 * ortho;

                            x = x - shift(1);
                            y = y - shift(2);

                            plot(x, y, 'Color', 'w', 'LineWidth', 2);
                        end
                    end
                end
            end
            
            % Pedestrain crossing
            ise = evalin( 'base', 'exist(''ped_xings'',''var'') == 1' );
            if (ise)
                ped_xings = evalin('base', 'ped_xings');
                for i = 1:size(ped_xings, 2)
                    ped_xing = ped_xings(i);
                    
                    mid_vector = ped_xing.end - ped_xing.start;

                    if (isfield(ped_xing, 'width') && ~isempty(ped_xing.width))
                        width = ped_xing.width;
                    else
                        width = 0.025;
                    end

                    % Get unit orthonormal
                    ortho = [-mid_vector(2), mid_vector(1)];
                    ortho = ortho / norm(ortho) * width;

                    top = floor(norm(mid_vector) * 75);
                    
                    for j = 0:2:top
                        p1 = mid_vector / top * j + ped_xing.start;
                        p2 = mid_vector / top * (j + 1) + ped_xing.start;
                        
                        s1 = p1 + ortho;
                        s2 = p2 + ortho;
                        s3 = p2 - ortho;
                        s4 = p1 - ortho;
                        
                        bg = polyshape([s1; s2; s3; s4]);
                        pbg = plot(bg);
                        pbg.LineStyle = 'none';
                        pbg.FaceColor = 'w';
                        pbg.FaceAlpha = 1;
                    end
                end
            end
            
            % Get intersection info from the base workspace.
            obj.N_Signals = evalin('base', 'num_signals');
            
            tlPositions = evalin('base', 'tlPositions');
            labelPositions = evalin('base', 'labelPositions');
            
            red = evalin('base', 'red');
            green = evalin('base', 'green');
            yellow = evalin('base', 'yellow');
            amber = evalin('base', 'amber');
            
            % Assert sizes
            assert(isequal(size(tlPositions), [obj.N_Signals 2]), 'Error, traffic light position must be a [num_signals x 2] matrix');
            assert(isequal(size(labelPositions), [obj.N_Signals 2]), 'Error, label positions must be a [num_signals x 2] matrix');
            
            assert(isequal(size(red), [obj.N_Signals 1]), 'Error, red vector must be a column vector of size num_signals');
            assert(isequal(size(green), [obj.N_Signals 1]), 'Error, green vector must be a column vector of size num_signals');
            assert(isequal(size(yellow), [obj.N_Signals 1]), 'Error, yellow vector must be a column vector of size num_signals');
            assert(isequal(size(amber), [obj.N_Signals 1]), 'Error, amber vector must be a column vector of size num_signals');
            
            % Allocate an array to store graphics handles.
            obj.TrafficLightGraphicObjects = gobjects(obj.N_Signals, 1);
            obj.LabelGraphicObjects = gobjects(obj.N_Signals, 1);
            
            % Plot traffic lights.
            for i = 1:obj.N_Signals
                obj.TrafficLightGraphicObjects(i) = plot(tlPositions(i,1), tlPositions(i,2), ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', IntersectionPlotter.getColor(red(i), green(i), yellow(i), amber(i)), ...
                    'MarkerEdgeColor', 'k', ...
                    'MarkerSize', 10, ...
                    'Parent', obj.AxesHandle);
            end
            
            % Plot labels.
            for i = 1:obj.N_Signals
                txt = {sprintf('Arrival Rate: %0.3f', 0), sprintf('Departure Rate: %0.3f', 0), sprintf('Queue Size: %d', 0)};
                obj.LabelGraphicObjects(i) = text(labelPositions(i,1), labelPositions(i,2), txt);
            end
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj, red, green, yellow, amber, queueSizes, arr_rates, dep_rates)
            assert(isequal(size(arr_rates), [obj.N_Signals 1]), "Error, arrival rate vector must match number of labels.");
            assert(isequal(size(dep_rates), [obj.N_Signals 1]), "Error, departure rate vector must match number of labels.");
            assert(isequal(size(queueSizes), [obj.N_Signals 1]), 'Error, queue size vector must match number of labels.');
            
            assert(isequal(size(red), [obj.N_Signals 1]), 'Error, red vector must be a column vector of size num_signals');
            assert(isequal(size(green), [obj.N_Signals 1]), 'Error, green vector must be a column vector of size num_signals');
            assert(isequal(size(yellow), [obj.N_Signals 1]), 'Error, yellow vector must be a column vector of size num_signals');
            assert(isequal(size(amber), [obj.N_Signals 1]), 'Error, amber vector must be a column vector of size num_signals');
            
            if (isvalid(obj.AxesHandle))
                % Traffic lights
                for i = 1:size(obj.TrafficLightGraphicObjects)
                	obj.TrafficLightGraphicObjects(i).MarkerFaceColor = IntersectionPlotter.getColor(red(i), green(i), yellow(i), amber(i));
                end
                
                % Labels
                for i = 1:size(obj.LabelGraphicObjects)
                    txt = {sprintf('Arrival Rate: %0.3f', arr_rates(i)), sprintf('Departure Rate: %0.3f', dep_rates(i)), sprintf('Queue Size: %d', queueSizes(i))};
                    set(obj.LabelGraphicObjects(i), 'String', txt);
                end
                
                % Limits the number of updates to 20 frames per second.
                drawnow limitrate
            end
        end
    end
    
    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog.
            simMode = "Interpreted execution";
        end
        
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block
            % dialog.
            flag = false;
        end
        
        function [name1, name2, name3, name4, name5, name6, name7] = getInputNamesImpl(~)
            % Return input port names for System block.
            name1 = 'Green';
            name2 = 'Red';
            name3 = 'Yellow';
            name4 = 'Amber';
            name5 = 'QueueSizes';
            name6 = 'ArrRates';
            name7 = 'DepRates';
        end
        
        function color = getColor(red, green, yellow, ~)
            if (red == 1)
                color = '#ff0000';
            elseif (green == 1)
                color = 'g';
            elseif (yellow == 1)
                color = '#ffff00';
            else
                color = [1 0.5 0];
            end
        end
    end
end

