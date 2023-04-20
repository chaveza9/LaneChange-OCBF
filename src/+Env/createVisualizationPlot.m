function [figScene, lineHandles] = createVisualizationPlot(scenarioObj,params,vehicleName)
%exampleHelperInitializeSimulator Set up scenario and scenario viewer
    arguments
        scenarioObj
        params struct
        vehicleName (1,:) char {mustBeText, mustBeNonzeroLengthText} = 'VehC';
    end
    % Reset the scenario
    restart(scenarioObj)
    close all;
    % Check if there are actors available
    if isempty(scenarioObj.Actors)
        error('actors need to be created first and added to the scenario object')
    end
    % Choose a random VehC Actor on Simulation
   Vehicle = scenarioObj.Actors(strcmp([scenarioObj.Actors.Name],vehicleName));
   if isempty(Vehicle)
       error('No vehicle with name %s has been found',vehicleName)
   end
    %% Create Visualization
    % Create a chasePlot using the visualization scenario.
    figScene = figure('Name','Driving Scenario','Tag','ScenarioGenerationDemoDisplay');
    figScene.Visible = 'on';
    set(figScene,'Position',[0, 0, 1032, 1032]);
    movegui(figScene,'center');
    
    % Add the chase plot
    hCarViewPanel = uipanel(figScene,'Position',[0.5 0 0.5 1],'Title','Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(Vehicle,'ViewLocation',-[params.actors.carLen*6, 0],'ViewHeight',15,'ViewPitch',20, 'Parent',hCarPlot,'Meshes','on');

    % Add annotations to chase plot
    annotationSpacing = 0.05;
    x = [.9 .95];
    y = [.95 .95];
    bdims = [x(1)-.175 ...
             y(1)-(annotationSpacing*3.5) ...
             (x(2)-x(1))+.2 ...
             annotationSpacing*4];
    p = uipanel(figScene,'Position',bdims,'BackgroundColor',[.75 .75 .75]);
    commonProps = {'VerticalAlignment','middle','HorizontalAlignment',...
        'center','Margin',0,'FontUnits','normalized','Color','k'};
    textBoxLocation = [.05 .725 .9 .15];
    lineHandles = [];
    for i = 1:length(scenarioObj.Actors)
        a = annotation(p,'textbox',textBoxLocation,...
            'String',scenarioObj.Actors(i).Name,...
            'EdgeColor',scenarioObj.Actors(i).PlotColor,...
            'BackgroundColor',scenarioObj.Actors(i).PlotColor,...
            'LineWidth',2,commonProps{:});
        textBoxLocation(2) = textBoxLocation(2) - (1-.25)/length(scenarioObj.Actors);
        lineHandles = cat(1,lineHandles,a);
    end

    % Add the top view of the generated scenario
    hViewPanel = uipanel(figScene,'Position',[0 0 0.5 1],'Title','Top View');
    hCarPlot = axes(hViewPanel);
    chasePlot(Vehicle,'ViewLocation',-[params.actors.carLen*3, 0],'ViewHeight',300,'ViewPitch',90, 'Parent',hCarPlot,'Meshes','on');
   

end % function
