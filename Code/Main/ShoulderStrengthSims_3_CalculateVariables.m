function ShoulderStrengthSims_3_CalculateVariables(taskName,nMeshPoints,whichModels,resultsDir,modelDir)
    
    %Function takes the MoCo printed solutions of the chosen task, collates
    %the results from different models and calculates the output variables
    %of interest - subsequently saving thes a Matlab output for later use.
    %
    %Current variables included in this process are:
                % Glenohumeral kinematics
                % Joint reaction forces (components and resultant)
                % Glenohumeral stability metric
                % Muscle activations and forces
                % Muscle analysis data (e.g. fiber lengths, velocities)
                % Muscle cost
    %
    %INPUTS:    taskName = string containing appropriate task name
        % Current options are:
                % ConcentricAxillaTouch
                % ConcentricForwardReach
                % ConcentricHairTouch
                % ConcentricRearTouch
                % ConcentricUpwardReach90
                % ConcentricUpwardReach105
                
    %           nMeshPoints = number referring to the desired mesh points result to import
    %           whichModels = string of 'all' 'weakened' or 'strengthened' of which set of muscles to run
    %           modelDir = full path to directory containing models 
    
    import org.opensim.modeling.*
    warning off
    home_dir = cd;
    
    %% Identify the models to iterate through for predictive simulations
    
    %Navigate to simulation results directory
    cd(resultsDir); cd(taskName);
    
    %Get folder names for the models that have been simulated
    f = dir;
    for ff = 3:length(f)    %ignore first two 'blank' entries
        modelsRun{ff-2,1} = f(ff).name;
    end
    clear ff f
    
    %Grab out the number for each filename representing strength value
    for ff = 1:length(modelsRun)
        S = sprintf('%s ', modelsRun{ff});
        letters = (S < '0' | S > '9');  % Mask all non-numbers
        S(letters) = ' ';
        strengthVal(ff,1) = str2double(S);
        clear S letters
    end
    clear ff
    
    %Identify models to run simulations with based on input
    switch whichModels        
        
        case 'weakened'
            %Only grab models with reduced strength
            %Find model files with number < 100 in name
            %Create logical for values less than 100. Also include NaN here
            %for the baseline model
            for ss = 1:length(strengthVal)
                if strengthVal(ss) < 100 || isnan(strengthVal(ss))
                    grabModel(ss,1) = true;
                else
                    grabModel(ss,1) = false;
                end
            end
            %Allocate model names
            modelNames = modelsRun(grabModel);
            
        case 'strengthened'
            %Only grab models with reduced strength
            %Find model files with number < 100 in name
            %Create logical for values less than 100. Also include NaN here
            %for the baseline model
            for ss = 1:length(strengthVal)
                if strengthVal(ss) > 100 || isnan(strengthVal(ss))
                    grabModel(ss,1) = true;
                else
                    grabModel(ss,1) = false;
                end
            end
            %Allocate model names
            modelNames = modelsRun(grabModel);
            
        case 'all'
            %Allocate model names
            modelNames = modelsRun;
            
    end
    
    %% Loop through and extract data / run analyses from MoCo outputs
    
    %Loop through models
    for m = 1:length(modelNames)
        
        %% Navigate to results directory
        cd(modelNames{m});
        
        %Load in the final iteration solution
        D = importdata([modelNames{m},'_',taskName,'_MuscleDriven_',num2str(nMeshPoints),'meshPoints_solution.sto']);
        
        %Find the column indices corresponding to different measures
        
        %Joint angles
        jointAngleInd = logical(contains(D.colheaders,'/jointset')) & logical(contains(D.colheaders,'/value'));
        %Joint speeds
        jointSpeedInd = logical(contains(D.colheaders,'/jointset')) & logical(contains(D.colheaders,'/speed'));
        %Joint accelerations
        jointAccelInd = logical(contains(D.colheaders,'/jointset')) & logical(contains(D.colheaders,'/accel'));
        %Muscle activations
        muscleActInd = logical(contains(D.colheaders,'/forceset')) & logical(contains(D.colheaders,'/activation'));

        %Extract data
        
        %Joint angles
        Results.(modelNames{m}).(char(taskName)).kinematics.time = D.data(:,1);
        %Setup blank variable for inputting joint angle names
        jointAngles = [];
        for k = 1:length(jointAngleInd)
            if jointAngleInd(k)
                %Extract joint angle data
                
                %Extract the name of the joint angle. This will be the 4th
                %string split by a '/'
                j = strsplit(D.colheaders{k},'/');
                jointAngles{length(jointAngles)+1} = j{4};
                
                %Extract the data to relevant structure
                Results.(modelNames{m}).(char(taskName)).kinematics.(j{4}) = D.data(:,k);
                
                %Cleanup
                clear j
                
            else
                %don't extract data
            end
        end
        clear k
        
        %Joint velocities
        Results.(modelNames{m}).(char(taskName)).velocities.time = D.data(:,1);
        %Setup blank variable for inputting joint angle names
        jointVelocities = [];
        for k = 1:length(jointSpeedInd)
            if jointSpeedInd(k)
                %Extract joint angle data
                
                %Extract the name of the joint angle. This will be the 4th
                %string split by a '/'
                j = strsplit(D.colheaders{k},'/');
                jointVelocities{length(jointVelocities)+1} = j{4};
                
                %Extract the data to relevant structure
                Results.(modelNames{m}).(char(taskName)).velocities.(j{4}) = D.data(:,k);
                
                %Cleanup
                clear j
                
            else
                %don't extract data
            end
        end
        clear k
        
        %Joint accelerations
        Results.(modelNames{m}).(char(taskName)).accelerations.time = D.data(:,1);
        %Setup blank variable for inputting joint angle names
        jointAccelerations = [];
        for k = 1:length(jointAccelInd)
            if jointAccelInd(k)
                %Extract joint angle data
                
                %Extract the name of the joint angle. This will be the 4th
                %string split by a '/'
                j = strsplit(D.colheaders{k},'/');
                jointAccelerations{length(jointAccelerations)+1} = j{4};
                
                %Extract the data to relevant structure
                Results.(modelNames{m}).(char(taskName)).accelerations.(j{4}) = D.data(:,k);
                
                %Cleanup
                clear j
                
            else
                %don't extract data
            end
        end
        clear k
        
        %Muscle activations
        Results.(modelNames{m}).(char(taskName)).activations.time = D.data(:,1);
        %Setup blank variable for inputting joint angle names
        muscleNames = [];
        for k = 1:length(muscleActInd)
            if muscleActInd(k)
                %Extract joint angle data
                
                %Extract the name of the joint angle. This will be the 3rd
                %string split by a '/'
                j = strsplit(D.colheaders{k},'/');
                muscleNames{length(muscleNames)+1} = j{3};
                
                %Extract the data to relevant structure
                Results.(modelNames{m}).(char(taskName)).activations.(j{3}) = D.data(:,k);
                
                %Cleanup
                clear j
                
            else
                %don't extract data
            end
        end
        clear k
        
        %% Run analyses on Moco output

        %Calculate muscle forces and muscle analysis

        %Initialise a blank analysis tool
        AnTool = AnalyzeTool();

        %Provide inputs to analysis tool
        %Set the model file
        AnTool.setModelFilename([modelDir,'\',modelNames{m},'.osim']);
% % %         AnTool.setModel(Model([modelDir,'\',modelNames{m},'.osim']));
        %Set the states file
        AnTool.setStatesFileName([pwd,'\',modelNames{m},'_',taskName,'_MuscleDriven_',num2str(nMeshPoints),'meshPoints_solution.sto']);
% % %         AnTool.setStatesFileName([modelNames{m},'_',taskName,'_MuscleDriven_',num2str(nMeshPoints),'meshPoints_solution.sto']);
        %Set time range from start and end time points in states file
        AnTool.setStartTime(D.data(1,1)); AnTool.setFinalTime(D.data(end,1));
        %Set results directory
        AnTool.setResultsDir(pwd);
        %Set tool name
        AnTool.setName('Analyse');

        %Intialise force reporter analysis set
        FrAnalysis = ForceReporter();

        %Provide inputs to force reporter analysis
        %Set name
        FrAnalysis.setName('ForceReporter');
        %Set start and end time
        FrAnalysis.setStartTime(D.data(1,1)); FrAnalysis.setEndTime(D.data(end,1));
        %Options
        FrAnalysis.setStepInterval(1);
        FrAnalysis.setInDegrees(true);

        %Add the force reporter analysis to the analyse tool
        AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
        
        %Initialise muscle analysis tool
        MaAnalysis = MuscleAnalysis();
        
        %Provide inputs to muscle analysis
        %Set name
        MaAnalysis.setName('MuscleAnalysis');
        %Set start and end time
        MaAnalysis.setStartTime(D.data(1,1)); MaAnalysis.setEndTime(D.data(end,1));
        %Options
        MaAnalysis.setStepInterval(1);
        MaAnalysis.setInDegrees(true);
        
        %Add the muscle analysis to the analyse tool
        AnTool.getAnalysisSet().cloneAndAppend(MaAnalysis);
        
        %Print tool to file
        % % % AnTool.dump()        
        AnTool.print('SetupAnalysis_ForceReporter_MuscleAnalysis.xml'); %needs short name due to long file paths

        %Run force reporter and muscle analysis tool
        clc
        system('opensim-cmd run-tool SetupAnalysis_ForceReporter_MuscleAnalysis.xml');
% % %         AnTool.run()
        
% % %         %Need to edit the force reporter output to remove the ligament and
% % %         %potential energy forces for joint reaction analysis to work
% % %         
% % %         %Load the force reporter results
% % %         frResults = load_sto_file([modelNames{m},'_',taskName,'_Analyse_ForceReporter_forces.sto']);
% % %         
% % %         %Extract just the time and muscle data
% % %         frNew.time = frResults.time;
% % %         for p = 1:length(muscleNames)
% % %             frNew.(muscleNames{p}) = frResults.(muscleNames{p});
% % %         end
% % %         clear p
% % %         
% % %         %Write over original forces file
% % %         write_sto_file(frNew,[modelNames{m},'_',taskName,'_Analyse_ForceReporter_forces.sto'],'no');
% % %         
% % %         %Cleanup
% % %         clear frResults frNew        
        
        %Joint reaction forces

        %Clear analysis set from original tool
        AnTool.getAnalysisSet().clearAndDestroy();

        %Initialise a joint reactions analysis set
        JrfAnalysis = JointReaction();

        %Provide inputs to joint reaction analysis
        %Set name
        JrfAnalysis.setName('JointReactions');
        %Set start and end time
        JrfAnalysis.setStartTime(D.data(1,1)); JrfAnalysis.setEndTime(D.data(end,1));
        %Set forces file
        JrfAnalysis.setForcesFileName([pwd,'\Analyse_ForceReporter_forces.sto']);
        %Set joint names
        jnt = ArrayStr(); jnt.append('unrothum'); jnt.append('shoulder2');
        JrfAnalysis.setJointNames(jnt);
        %Set bodies to apply on
        bod = ArrayStr(); bod.append('parent'); bod.append('child');
        JrfAnalysis.setOnBody(bod);
        %Set frames to express in
        frm = ArrayStr(); frm.append('scapula'); frm.append('humerus');
        JrfAnalysis.setInFrame(frm);
        %Options
        JrfAnalysis.setStepInterval(1);
        JrfAnalysis.setInDegrees(true);

        %Add the joint reaction analysis to the analyse tool
        AnTool.getAnalysisSet().cloneAndAppend(JrfAnalysis);
        
        %Print tool to file
        % % % AnTool.dump()
        AnTool.print('SetupAnalysis_JointReaction.xml'); %needs short name due to long file paths

        %Run joint reaction tool
        clc
        system('opensim-cmd run-tool SetupAnalysis_JointReaction.xml');
           
        %Cleanup
        clear AnTool bod frm jnt JrfAnalysis FrAnalysis MaAnalysis
                
        %% Process analysis outputs
        
        %% Joint reaction force data
        
        %Load in joint reaction force data
        jrfData = importdata('Analyse_JointReactions_ReactionLoads.sto');
        
        %Grab the joint reaction forces applied to the scapula
        %These will be under the headers of unrothum_on_scapula_in_scapula_f*
        Results.(modelNames{m}).(char(taskName)).joint_reactions.time = jrfData.data(:,1); 
        for j = 1:length(jrfData.colheaders)
           if strcmp(jrfData.colheaders{j},'unrothum_on_scapula_in_scapula_fx')
               Results.(modelNames{m}).(char(taskName)).joint_reactions.FX = jrfData.data(:,j);               
           elseif strcmp(jrfData.colheaders{j},'unrothum_on_scapula_in_scapula_fy')
               Results.(modelNames{m}).(char(taskName)).joint_reactions.FY = jrfData.data(:,j); 
           elseif strcmp(jrfData.colheaders{j},'unrothum_on_scapula_in_scapula_fz')
               Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ = jrfData.data(:,j); 
           else
               %don't take data
           end
        end
        clear j
        
        %Cleanup
        clear jrfData
        
        %Rotate the joint reaction force data to the rotated scapula frame

        %First need to get the frame that the data is expressed in
        %0 = scapula_offset; 2 = rotated frame
        osimModel = Model([modelDir,'\',modelNames{m},'.osim']);
        osimModel_state = osimModel.initSystem();
        origFrame = osimModel.getJointSet().get('unrothum').get_frames(0);
        rotFrame = osimModel.getJointSet().get('unrothum').get_frames(2);
        
        %Loop through the forces, express them as Vec3 and transform to the
        %rotated fossa frame. The express argument requires a model state.
        for i = 1:length(Results.(modelNames{m}).(char(taskName)).joint_reactions.time)
            currVec = Vec3(Results.(modelNames{m}).(char(taskName)).joint_reactions.FX(i,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.FY(i,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ(i,1));
            newVec = origFrame.expressVectorInAnotherFrame(osimModel_state,currVec,rotFrame);
            %Allocate to relevant structures
            Results.(modelNames{m}).(char(taskName)).joint_reactions.FX(i,1) = newVec.get(0);
            Results.(modelNames{m}).(char(taskName)).joint_reactions.FY(i,1) = newVec.get(1);
            Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ(i,1) = newVec.get(2);
            %Cleanup
            clear newVec currVec
        end
        clear i
        
        %Cleanup
        clear origFrame rotFrame osimModel osimModel_state
        
        %Calculate resultant JRF force vector
        for i = 1:length(Results.(modelNames{m}).(char(taskName)).joint_reactions.time)
            Results.(modelNames{m}).(char(taskName)).joint_reactions.FXYZ(i,1) = ...
                sqrt((Results.(modelNames{m}).(char(taskName)).joint_reactions.FX(i,1)^2) + ...
                (Results.(modelNames{m}).(char(taskName)).joint_reactions.FY(i,1)^2) + ...
                (Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ(i,1)^2));
        end
        clear i

        %Now that we have the vectors in the appropriate frames we can identify the
        %resultant vector and the relevant 3D angles of the resultant

        %Calculate the 3D angles theta and phi
        for i = 1:length(Results.(modelNames{m}).(char(taskName)).joint_reactions.time)
            Results.(modelNames{m}).(char(taskName)).joint_reactions.spherPhi(i,1) = ...
                atand((Results.(modelNames{m}).(char(taskName)).joint_reactions.FX(i,1) / ...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.FY(i,1)));
            Results.(modelNames{m}).(char(taskName)).joint_reactions.spherTheta(i,1) = ...
                acosd((Results.(modelNames{m}).(char(taskName)).joint_reactions.FZ(i,1) / ...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.FXYZ(i,1)));
        end
        clear i
        
        %With these angles and a proposed prescribed distance along the y-axis of
        %this frame, we can work out what the distances from the centre of the
        %glenoid fossa is. Values will be in the same notation as the frame, with
        %positive being up and forward, and negative being down and back.

        %We can set a distance based on a generic radius of the humeral head.
        %Boileau & Walch (1997) found a diameter of 46.2mm for the humeral head;
        %and Knowles et al. (2016) found a diameter of 49mm for the humeral head,
        %slightly bigger and smaller for males and females respectively (but not
        %reported in text, only figure?).

        %Set the distance along the Y-axis to the glenoid fossa based on the dat of
        %Knowles et al. (2016) - note it is a radius rather than diameter
        dY = 49/2;

        %Calculate distances
        for i = 1:length(Results.(modelNames{m}).(char(taskName)).joint_reactions.time)
            Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos(i,1) = ...
                tand(Results.(modelNames{m}).(char(taskName)).joint_reactions.spherPhi(i,1)) * dY;
            Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos(i,1) = ...
                tand((90-Results.(modelNames{m}).(char(taskName)).joint_reactions.spherTheta(i,1))) * dY;
        end
        clear i

% % %         %We can now attempt to plot a trace of these results on an ellipse that
% % %         %uses the dimensions of generic scapula anatomy. von Schroeder et al.
% % %         %(2001) reported an AP diameter of 28.6, 25.8 and 30.9 in combined, females
% % %         %and males, respectively; and a SI length of 36.5, 33.6 and 38.0 for
% % %         %combined, females and males, respectively. We'll go with the combined data
% % %         %for now...
% % %         drawEllipse(0,0,28.6/2,36.5/2);
% % %         %Edit the properties of the line object
% % %         h = findobj(gca,'Type','line');
% % %         h.Color = 'k'; h.LineWidth = 2;
% % %         axis equal
% % %         hold on
% % %         
% % %         %Draw the pattern of the original and altered models point of application
% % %         plot(Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos,...
% % %             Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos,'r','LineWidth',2);
% % %         
% % %         %Close plot
% % %         close all; clear h

        %Calculate GH stability using JRF data
        for g = 1:length(Results.(modelNames{m}).(char(taskName)).joint_reactions.time)
            %Find point on the glenoid rim that the vector would intersect in the 2D
            %space - the angles used to reach this point from the GHJ centre will then
            %be used in the calculation of GH stability. This uses a custom function
            %where the inputs are the major axis (i.e. AP diameter/2), minor axis (SI
            %diameter/2), centre coordinates of ellipse, starting point of line (i.e.
            %centre of ellipse in this case), and end point of line (projected point on
            %glenoid).
            [C1,C2] = lineEllipse(28.6/2,36.5/2,[0,0],[0,0],...
                [Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos(g,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos(g,1)]);
            %Need to determine which intersection point is required for the
            %calculations, this can be determined by checking the distance between the
            %points and taking whichever is closer.
            C1_dist = dist_markers2D([Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos(g,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos(g,1)],C1);
            C2_dist = dist_markers2D([Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos(g,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos(g,1)],C2);
            if C2_dist <= C1_dist
                %Use C2 point
                ellipseEdge_horz = C2(1); ellipseEdge_vert = C2(2);
            else
                %Use C1 point
                ellipseEdge_horz = C1(1); ellipseEdge_vert = C1(2);
            end
            clear C1 C2 C1_dist C2_dist

            %Calculate relative distance of projected point from glenoid centre to
            %ellipse edge point and use this as GH stability value
            Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab(g,1) = ...
                dist_markers2D([Results.(modelNames{m}).(char(taskName)).joint_reactions.horzPos(g,1),...
                Results.(modelNames{m}).(char(taskName)).joint_reactions.vertPos(g,1)],[0,0]) / ...
                dist_markers2D([ellipseEdge_horz,ellipseEdge_vert],[0,0]);
            
            %Cleanup
            clear ellipseEdge_horz ellipseEdge_vert
            
        end
        clear g
        
        %Calculate mean and peak GH stability values
        Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab_mean = ...
            mean(Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab);
        Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab_peak = ...
            max(Results.(modelNames{m}).(char(taskName)).joint_reactions.GHstab);
        
        %% Muscle force data
        
        %Load in muscle force data
        mfData = importdata('Analyse_ForceReporter_forces.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).muscle_forces.time = mfData.data(:,1);
        for v = 2:length(mfData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).muscle_forces.(mfData.colheaders{v}) = mfData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear mfData
        
        %% Muscle analysis data
              
        %Load in fibre length data
        flData = importdata('Analyse_MuscleAnalysis_FiberLength.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).fibre_lengths.time = flData.data(:,1);
        for v = 2:length(flData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).fibre_lengths.(flData.colheaders{v}) = flData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear flData
        
        %Load in muscle length data
        mlData = importdata('Analyse_MuscleAnalysis_Length.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).muscle_lengths.time = mlData.data(:,1);
        for v = 2:length(mlData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).muscle_lengths.(mlData.colheaders{v}) = mlData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear mlData
        
        %Load in fibre velocity data
        fvData = importdata('Analyse_MuscleAnalysis_FiberVelocity.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).fibre_velocities.time = fvData.data(:,1);
        for v = 2:length(fvData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).fibre_velocities.(fvData.colheaders{v}) = fvData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear fvData
        
        %Load in normalised fibre length data
        flData = importdata('Analyse_MuscleAnalysis_NormalizedFiberLength.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).fibre_lengths_norm.time = flData.data(:,1);
        for v = 2:length(flData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).fibre_lengths_norm.(flData.colheaders{v}) = flData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear flData
        
        %Load in normalised fibre velocity data 
        fvData = importdata('Analyse_MuscleAnalysis_NormFiberVelocity.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).fibre_velocities_norm.time = fvData.data(:,1);
        for v = 2:length(fvData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).fibre_velocities_norm.(fvData.colheaders{v}) = fvData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear fvData
        
        %Load in passive force data 
        pfData = importdata('Analyse_MuscleAnalysis_PassiveFiberForce.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).passive_fibre_force.time = pfData.data(:,1);
        for v = 2:length(pfData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).passive_fibre_force.(pfData.colheaders{v}) = pfData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear pfData
        
        %Load in active force data 
        afData = importdata('Analyse_MuscleAnalysis_ActiveFiberForce.sto');
        
        %Loop through and allocate to structures
        %The names for columns in this correspond to the muscle names so
        %there is no need to mess around with these
        Results.(modelNames{m}).(char(taskName)).active_fibre_force.time = afData.data(:,1);
        for v = 2:length(afData.colheaders) %skip time variable by starting at 2
            Results.(modelNames{m}).(char(taskName)).active_fibre_force.(afData.colheaders{v}) = afData.data(:,v);
        end
        clear v
        
        %Cleanup
        clear afData
        
        %Calculate 'cost' for each muscle taking into account it's
        %instantaneous muscle force, and maximum force capacity considering
        %it's instantaneous normalised length and velocity. See van der
        %Krogt et al. (2012), Gait Posture, 36, 113-119 for equation.
        %Chosen to use active muscle force as the measure of muscle force
        %to ignore the influence of passive force.
        Results.(modelNames{m}).(char(taskName)).muscle_cost.time = ...
            Results.(modelNames{m}).(char(taskName)).muscle_forces.time(:,1);
        %Intialise model
        osimModel = Model([modelDir,'\',modelNames{m},'.osim']);
        %Loop through muscles to calculate cost
        for v = 1:length(muscleNames)           
            %Get the current muscles force velocity and force length curves
            %First get the muscle in Millard form
            MillardMuscle = Millard2012EquilibriumMuscle.safeDownCast(osimModel.getMuscles.get(muscleNames{v}));
            %Get force length curve
            FLcurve = MillardMuscle.getActiveForceLengthCurve();
            %Get force velocity curve
            FVcurve = MillardMuscle.getForceVelocityCurve();
            %Get maximum isometric force of the muscle
            Fmax = osimModel.getMuscles.get(muscleNames{v}).get_max_isometric_force();
            
            %Loop through data and calculate muscle cost
            for d = 1:length(Results.(modelNames{m}).(char(taskName)).muscle_cost.time)
                %Get the FL multiplier based on the current normalised fibre length
                FLx = FLcurve.calcValue(Results.(modelNames{m}).(char(taskName)).fibre_lengths_norm.(muscleNames{v})(d,1));
                %Get the FV multiplier based on the current normalised fibre velocity
                FVx = FVcurve.calcValue(Results.(modelNames{m}).(char(taskName)).fibre_velocities_norm.(muscleNames{v})(d,1));
                %Calculate muscle cost for the current time step
                Results.(modelNames{m}).(char(taskName)).muscle_cost.(muscleNames{v})(d,1) = ...
                    (Results.(modelNames{m}).(char(taskName)).active_fibre_force.(muscleNames{v})(d,1) / (Fmax * FLx * FVx))^2;
                %Cleanup
                clear FLx FVx                
            end
            clear d
            
            %Cleanup
            clear MillardMuscle FLcurve FVcurve Fmax
        end
        clear v
        
        
        %% End model loop       
        
        %Cleanup
        clear D
        
        %Return to results directory for task
        cd([resultsDir,'\',taskName]);
        
    end
    clear m
        
        %% Save processed data to file
        
        %Navigate to appropriate directory
        cd('..\..\CompiledSimulations');
        
        %Save matlab database
        save([taskName,'_CompiledResults_',whichModels,'.mat'],...
            'jointAccelerations','jointAngles','jointVelocities','modelNames',...
            'muscleNames','nMeshPoints','Results','resultsDir','taskName');
        
        %Return to home directory
        cd(home_dir);
        
        %%
        
    %%%--- End of ShoulderStrengthSims_3_CalculateVariables.m ---%%%
    
end