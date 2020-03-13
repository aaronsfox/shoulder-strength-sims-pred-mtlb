function ShoulderStrengthSims_4_VisualiseResults(taskName,whichModels)
    
    %Function to grab the compiled resuls for a specific task and visualise
    %the results across the different models.
    %
    %INPUTS:    taskName = string containing appropriate task name
        % Current options are:
                % ConcentricAxillaTouch
                % ConcentricForwardReach
                % ConcentricHairTouch
                % ConcentricRearTouch
                % ConcentricUpwardReach90
                % ConcentricUpwardReach105

    %           whichModels = string of 'all' 'weakened' or 'strengthened' of which set of muscles to run
    
    import org.opensim.modeling.*
    warning off
    home_dir = cd;
    
    %%%%% TO DO: checks for inputs...
    
    %% Visualise results
    
    %Load in compiled data
    load([taskName,'_CompiledResults_',whichModels,'.mat']);
    
    %Generate plots of data
        
    %%%%% TO DO: find a place to store these figures

    %Glenohumeral kinematics

    %Initialise figure
    h = figure('units','normalized','outerposition',[0.1 0.1 0.8 0.4]);
    %Loop through models and plot
    for m = 1:length(modelNames)
        %Elevation plane
        subplot(1,3,1); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).kinematics.elv_angle(:,1)));
        %Elevation angle
        subplot(1,3,2); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).kinematics.shoulder_elv(:,1)));
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).kinematics.shoulder_rot(:,1)));
    end
    clear m
    %Add labels
    subplot(1,3,1); ylabel('Elevation Plane (deg)'); xlabel('Time (s)');
    subplot(1,3,2); ylabel('Elevation Angle (deg)'); xlabel('Time (s)');
    subplot(1,3,3); ylabel('Shoulder Rotation (deg)'); xlabel('Time (s)');
    %Legend
% % %         legend(modelNames,'Location','southeast');    %fix legend parameters
    %Save figure
    set(h,'PaperPositionMode','auto');
    saveas(h,fullfile(pwd,[taskName,'_',whichModels,'_GlenohumeralKinematics.fig']));
    saveas(h,fullfile(pwd,[taskName,'_',whichModels,'_GlenohumeralKinematics.png']));
    close all; clear h

    %Muscle activations

    %%%% perhaps see plot script in Wu Model files for a sample of how
    %%%% to adequately plot so many muscles...

    %Muscle forces

    %%%% perhaps see plot script in Wu Model files for a sample of how
    %%%% to adequately plot so many muscles...

    %%%%%%% these and any other plots are going to be messy with the
    %%%%%%% amount of models being plotted on top of one another...


    %Joint reaction forces

    %Initialise figure
    h = figure('units','normalized','outerposition',[0.1 0.1 0.8 0.4]);
    %Loop through models and plot
    for m = 1:length(modelNames)
        %Elevation plane
        subplot(1,3,1); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).joint_reactions.FX(:,1)));
        %Elevation angle
        subplot(1,3,2); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).joint_reactions.FY(:,1)));
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(Results.(modelNames{m}).(char(taskName)).kinematics.time(:,1),...
            rad2deg(Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ(:,1)));
    end
    clear m
    %Add labels

    %%%%% TO DO: fix labels

    subplot(1,3,1); ylabel('FX'); xlabel('Time (s)');
    subplot(1,3,2); ylabel('FY'); xlabel('Time (s)');
    subplot(1,3,3); ylabel('FZ'); xlabel('Time (s)');
    %Legend
% % %         legend(modelNames,'Location','southeast');    %fix legend parameters
    %Save figure
    set(h,'PaperPositionMode','auto');
    saveas(h,fullfile(pwd,[taskName,'_',whichModels,'_JointReactionForces.fig']));
    saveas(h,fullfile(pwd,[taskName,'_',whichModels,'_JointReactionForces.png']));
    close all; clear h
    
    
    %%
    
% % %     %Simple grab of mean and peak gh stability values
% % %     for m = 1:length(modelNames)
% % %         gh(m,1) = Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab_mean(1,1)
% % %         gh(m,2) = Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab_peak(1,1)
% % %     end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    