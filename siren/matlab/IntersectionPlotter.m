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
            
            % Get intersection info from the base workspace.
            intersectionInfo = evalin('base', 'intersectionInfo');
            
            tlPositions = intersectionInfo.tlPositions;
            nTrafficLigths = size(tlPositions);
            
            labelPositions = intersectionInfo.labelPositions;
            nLabelPositions = size(labelPositions);
            
            assert(nTrafficLigths == nLabelPositions, 'Error, number of labels must match number of traffic lights');
            
            % Allocate an array to store graphics handles.
            obj.TrafficLightGraphicObjects = gobjects(nTrafficLigths, 1);
            obj.LabelGraphicObjects = gobjects(nLabelPositions, 1);
            
            % Plot traffic lights.
            for i = 1:nTrafficLigths
                obj.TrafficLightGraphicObjects(i) = plot(tlPositions(i,1), tlPositions(i,2), ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', 'r', ...
                    'MarkerSize', 8, ...
                    'Parent', obj.AxesHandle);
            end
            
            % Plot labels.
            for i = 1:nLabelPositions
                txt = {sprintf('Rate: %0.5f', 0), sprintf('Queue Size: %d', 0)};
                obj.LabelGraphicObjects(i) = text(labelPositions(i,1), labelPositions(i,2), txt);
            end

            % set current figure handle position.
            set(gcf, 'Position', [1 1 800 600]);
            
            %TODO: Draw driving surface
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj, tlStates, queueSizes, rates)
            assert(size(queueSizes) == size(rates), "Error, queue size and rate vectors must have same length.");
            assert(size(tlStates) == size(obj.TrafficLightGraphicObjects), 'Error, number of traffic light states must match number of traffic lights.');
            assert(size(queueSizes) == size(obj.LabelGraphicObjects), 'Error, queue size vector must match number of labels.')
            
            if (isvalid(obj.AxesHandle))
                % Traffic lights
                for i = 1:size(obj.TrafficLightGraphicObjects)
                    switch(tlStates(i))
                        case 0
                            % Set the marker color to red.
                            obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'r';
                        case 1
                            % Set the marker color to amber.
                            obj.TrafficLightGraphicObjects(i).MarkerFaceColor = [1 0.5 0];
                        case 2
                            % Set the marker color to green.
                            obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'g';
                        otherwise
                            % Set the marker color to yellow.
                            obj.TrafficLightGraphicObjects(i).MarkerFaceColor = 'y';
                    end
                end
                
                % Labels
                for i = 1:size(obj.LabelGraphicObjects)
                    txt = {sprintf('Rate: %0.5f', rates(i)), sprintf('Queue Size: %d', queueSizes(i))};
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
        
        function [name1, name2, name3] = getInputNamesImpl(~)
            % Return input port names for System block.
            name1 = 'TrafficLightStates';
            name2 = 'QueueSizes';
            name3 = 'Rates';
        end
    end
end

