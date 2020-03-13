function [solution] = SetupRun_MoCo_ProblemSolver(ModelFile,taskName,taskBounds,nMeshPoints,outputFolder,muscleDriven,guessFile)

% INPUTS
    % nMeshPoints - number of mesh points
    % ModelFile - string containing full path to model used in simulation
    % taskName - string containing task name for simulation
        % Options are:
            % ConcentricAxillaTouch
            % ConcentricForwardReach
            % ConcentricHairTouch
            % ConcentricRearTouch
            % ConcentricUpwardReach90
            % ConcentricUpwardReach105

    % outputFolder - string containing full path to output folder
    % guessFile - solution .sto file for setting guess
    % muscleDriven - logical as to whether use a muscle or torque driven simulation
    % initialStates - .sto file used to set initial states for eccentric movements

% OUTPUTS
    % solution - the MoCo solution object for the predictive simulation

import org.opensim.modeling.*;

%% Model set-up

%Get fileparts of model
[~,modelName,~] = fileparts(ModelFile);

%Load the desired model
osimModel = Model(ModelFile);
% % % osimModel_state = osimModel.initSystem();

%Lock the thorax joints of the model to make this a shoulder only movement
coordSet = osimModel.updCoordinateSet();
coordSet.get('thorax_tilt').set_locked(true);
coordSet.get('thorax_list').set_locked(true);
coordSet.get('thorax_rotation').set_locked(true);
coordSet.get('thorax_tx').set_locked(true);
coordSet.get('thorax_ty').set_locked(true);
coordSet.get('thorax_tz').set_locked(true);

if muscleDriven
    %Add torque actuators to each degree of freedom.
    %Loop through and add coordinate actuators
    %Don't add anything for the thorax
    for c = 1:coordSet.getSize()
        if contains(char(coordSet.get(c-1).getName()),'thorax')
            %don't add a coordinate actuator as these coordinates won't move    
        elseif strcmp(char(coordSet.get(c-1).getName()),'elbow_flexion')
            %Add an idealised torque actuator.
            addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),300);
        elseif strcmp(char(coordSet.get(c-1).getName()),'pro_sup')
            addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),100);
        elseif strcmp(char(coordSet.get(c-1).getName()),'elv_angle') || ...
                strcmp(char(coordSet.get(c-1).getName()),'shoulder_elv') || ...
                strcmp(char(coordSet.get(c-1).getName()),'shoulder_rot')
            %Add reserve actuator with low optimal torque
            addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),2);
        end
    end
    clear c
else
    %Remove the muscles from the model
    osimModel.updForceSet().clearAndDestroy();
    osimModel.initSystem();
    %Add coordinate actuators to each degree of freedom to drive
    %Loop through and add coordinate actuators. Don't add anything for the thorax
    for c = 1:coordSet.getSize()
        if contains(char(coordSet.get(c-1).getName()),'thorax')
            %don't add a coordinate actuator as these coordinates won't move    
        else
            %Add an idealised torque actuator
            addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),300);
        end
    end 
end

%Add a 1kg mass to the hand for the reaching tasks
if strcmpi(taskName,'ConcentricUpwardReach90') || strcmpi(taskName,'EccentricUpwardReach90') || ...
        strcmpi(taskName,'ConcentricUpwardReach105') || strcmpi(taskName,'EccentricUpwardReach105') || ...
        strcmpi(taskName,'ConcentricForwardReach') || strcmpi(taskName,'EccentricForwardReach')        
    %Get hand mass and calculate added 1kg value
    newHandMass = osimModel.getBodySet().get('hand_r').getMass() + 1;
    %Set new hand mass
    osimModel.getBodySet().get('hand_r').setMass(newHandMass);
else
    %don't add mass to hand
end

%% Set-up general parameters

%If statement for whether to use the inverse solver for basic movements or
%the predictive solver for functional movements
if strcmp(taskName,'Abd0_90') || strcmp(taskName,'Abd90_180') || ...
        strcmp(taskName,'ExtRot0_90') || strcmp(taskName,'AbdExtRot0_90')
    
    %Replace the muscles in the model with muscles from DeGroote, Fregly, 
    %et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem 
    %Formulations for Solving the Muscle Redundancy Problem". These muscles
    %have the same properties as the original muscles but their characteristic
    %curves are optimized for direct collocation (i.e. no discontinuities, 
    %twice differentiable, etc).
    DeGrooteFregly2016Muscle().replaceMuscles(osimModel);
    
    %Turn off muscle-tendon dynamics to keep the problem simple.
    %This is probably already done in the model anyway
    for m = 0:osimModel.getMuscles().getSize()-1
        musc = osimModel.updMuscles().get(m);
        musc.set_ignore_tendon_compliance(true);
    end
    clear m
    
    %Set-up inverse solver
    inverse = MocoInverse();
    %Set kinematics
    inverse.setKinematicsFile(['X:\ShoulderInstability\MuscleStrengtheningSimulation\BasicKinematics\',taskName,'_Kinematics_q.sto']);
    %Set mesh interval
    inverse.set_mesh_interval(nMeshPoints);
    %Set cost function
    inverse.set_minimize_sum_squared_states(true);	%need this when you have both excitations (muscles) and activations (torques)?
    %Set tolerance
    inverse.set_tolerance(1e-4);
    %Set kinematics to allow extra columns
    inverse.set_kinematics_allow_extra_columns(true);
    %Set outputs to write
    inverse.append_output_paths('.*states');
    % % % inverse.append_output_paths('.*normalized_fiber_length');
    % % % inverse.append_output_paths('.*passive_force_multiplier');
    %Set ignore tendon compliance to true
    inverse.set_ignore_tendon_compliance(true);
    %Set model
    inverse.setModel(osimModel);
    
    %% Print tool to file

    %Set whether muscle or torque driven
    if muscleDriven
        simType = 'MuscleDriven';
    else
        simType = 'TorqueDriven';
    end

    cd(outputFolder);
    inverse.setName([modelName,'_',taskName,'_',simType,'_inverse']);
    inverse.print([modelName,'_',taskName,'_',simType,'_inverse.omoco']);
    
    %% Run predictive problem
    tic
    % % % clc
    disp('Beginning optimisation...');
    solution = inverse.solve();
    runtime = toc;
    if muscleDriven
        disp([modelName,' ',taskName,' muscle driven inverse problem with mesh interval of ',num2str(nMeshPoints),' complete. Total time = ', num2str(runtime/60),' minutes.']);
    else
        disp([modelName,' ',taskName,' torque driven inverse problem with mesh interval of ',num2str(nMeshPoints),' complete. Total time = ', num2str(runtime/60),' minutes.']);
    end
    %Print details to text file
    fid = fopen([modelName,'_',taskName,'_',simType,'_inverse_Details.txt'],'wt');
    if muscleDriven
        fprintf(fid,[modelName,' ',taskName,' muscle driven inverse problem with mesh interval of ',num2str(nMeshPoints),' complete. Total time = ', num2str(runtime/60),' minutes.']);
    else
        fprintf(fid,[modelName,' ',taskName,' torque driven inverse problem with mesh interval of ',num2str(nMeshPoints),' complete. Total time = ', num2str(runtime/60),' minutes.']);
    end
    fclose(fid);
    
    %Rename solution file
    movefile('MocoTool_solution.sto',[modelName,'_',taskName,'_',simType,'_inverse_solution.sto']);
        
else
    
    %Set up predictive solver

    %Set general aspects irrespective of task
    
    %Create a new MocoTool.
    moco = MocoTool();

    %Configure the solver.
    solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(nMeshPoints);
    solver.set_dynamics_mode('implicit');
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_solver('ipopt');
    solver.set_transcription_scheme('hermite-simpson');
    solver.set_enforce_constraint_derivatives(true);
    solver.set_optim_hessian_approximation('limited-memory');
    solver.set_optim_finite_difference_scheme('forward');
    solver.set_optim_ipopt_print_level(5);
    solver.set_verbosity(2);

    %Set bounds on the problem.
    problem = moco.updProblem();

    %Set the time bounds for the movement to occur. This will be strict for the
    %simplistic movements to ensure that they occur at an average of 45 degrees
    %per second, while the others will occur within 0.1-2.5 seconds (with the
    %added aspect of an end time cost added at the end)
    if strcmpi(taskName,'Abd0_90') || strcmpi(taskName,'Abd90_180') || ...
            strcmpi(taskName,'ExtRot0_90') || strcmpi(taskName,'AbdExtRot0_90')
        problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(2));
    else
        problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0.1,2.5));
    end

    %% Set-up task specific parameters
    %  NOTE: A number of task parameters come from data extraction from Vidt et
    %  al. work. These can be found in the Vidt_MovementValues.mat file

    %Get the index row for the current tasks values in the task
    %bounds structure
    if contains(taskName,'AxillaTouch')
        taskInd = find(strcmp(taskBounds.taskNames,'AxillaTouch'));
    elseif contains(taskName,'ForwardReach')
        taskInd = find(strcmp(taskBounds.taskNames,'ForwardReach'));
    elseif contains(taskName,'HairTouch')
        taskInd = find(strcmp(taskBounds.taskNames,'HairTouch'));
    elseif contains(taskName,'RearTouch')
        taskInd = find(strcmp(taskBounds.taskNames,'RearTouch'));
    elseif contains(taskName,'UpwardReach90')
        taskInd = find(strcmp(taskBounds.taskNames,'UpwardReach90'));
    elseif contains(taskName,'UpwardReach105')
        taskInd = find(strcmp(taskBounds.taskNames,'UpwardReach105'));
    end
    
    %Allocate bounds on values
    
    %Set whether to take concentric or eccentric values
    if contains(taskName,'Concentric')
        
        %Set parameters for concentric movements
        
        %Shoulder elevation
        charValue = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/speed'];
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_elv.min(taskInd)),deg2rad(taskBounds.shoulder_elv.max(taskInd))),...
            MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(taskBounds.shoulder_elv.con_LB(taskInd)),deg2rad(taskBounds.shoulder_elv.con_UB(taskInd))));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0),MocoFinalBounds(0));
        clear mn mx charValue charSpeed

        %Shoulder rotation
        charValue = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/speed'];
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_rot.min(taskInd)),deg2rad(taskBounds.shoulder_rot.max(taskInd))),...
            MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(taskBounds.shoulder_rot.con_LB(taskInd)),deg2rad(taskBounds.shoulder_rot.con_UB(taskInd))));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0),MocoFinalBounds(0));
        clear mn mx charValue charSpeed

        %Elevation plane
        charValue = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/speed'];
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.elv_angle.min(taskInd)),deg2rad(taskBounds.elv_angle.max(taskInd))),...
            MocoInitialBounds(deg2rad(taskBounds.elv_angle.min(taskInd)),deg2rad(taskBounds.elv_angle.max(taskInd))),...
            MocoFinalBounds(deg2rad(taskBounds.elv_angle.con_LB(taskInd)),deg2rad(taskBounds.elv_angle.con_UB(taskInd))));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0),MocoFinalBounds(0));
        clear mn mx charValue charSpeed

        %Elbow flexion
        charValue = ['/jointset/',char(coordSet.get('elbow_flexion').getJoint().getName()),'/',char(coordSet.get('elbow_flexion').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('elbow_flexion').getJoint().getName()),'/',char(coordSet.get('elbow_flexion').getName()),'/speed'];
        mn = coordSet.get('elbow_flexion').getRangeMin();
        if contains(taskName,'Reach')
            %Limit elbow flexion to 90 degrees
            mx = deg2rad(90);
        else
            %Leave as max attainable elbow flexion
            mx = coordSet.get('elbow_flexion').getRangeMax();
        end
        problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(0))
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0),MocoFinalBounds(0));
        clear mn mx charValue charSpeed

        %Forearm
        charValue = ['/jointset/',char(coordSet.get('pro_sup').getJoint().getName()),'/',char(coordSet.get('pro_sup').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('pro_sup').getJoint().getName()),'/',char(coordSet.get('pro_sup').getName()),'/speed'];
        mn = coordSet.get('pro_sup').getRangeMin();
        mx = coordSet.get('pro_sup').getRangeMax();
        problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(0))
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(0),MocoFinalBounds(0));
        clear mn mx charValue charSpeed
    
    elseif contains(taskName,'Eccentric')
        
        %Access the final solution for the concetric portion of the chosen
        %movement. Can find the directory of this by replacing Eccentric
        %within the outputfolder
        
        %Find concentric output folder
        conFolder = strrep(outputFolder,'Eccentric','Concentric');
        cd(conFolder);
        
        %Find the final solution file for the concentric movement. This
        %will be the one with the largest mesh point number in filename
        
        %Find solution files
        solFiles = dir('*_solution.sto');
        %Find the file with the largest file size. This will correspond to
        %the densest solution
        for f = 1:length(solFiles)
            bytes(f,1) = solFiles(f).bytes;
        end
        solInd = find(bytes == max(bytes));
        solFilename = solFiles(solInd).name;
        
        %Load in solution file
        solD = importdata(solFilename);

        %Set parameters for eccentric movements. This includes setting the
        %initial bounds to be the final state of the concentric solution
        
        %Shoulder elevation
        charValue = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/speed'];
        charValueSolLog = logical(strcmp(charValue,solD.colheaders)); charValueSolInd = find(charValueSolLog);
        charSpeedSolLog = logical(strcmp(charSpeed,solD.colheaders)); charSpeedSolInd = find(charSpeedSolLog);
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_elv.min(taskInd)),deg2rad(taskBounds.shoulder_elv.max(taskInd))),...
            MocoInitialBounds(solD.data(end,charValueSolInd)),MocoFinalBounds(0));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(solD.data(end,charSpeedSolInd)),MocoFinalBounds(0));
        clear mn mx charValue charSpeed charValueSolLog charValueSolInd charSpeedSolLog charSpeedSolInd

        %Shoulder rotation
        charValue = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/speed'];
        charValueSolLog = logical(strcmp(charValue,solD.colheaders)); charValueSolInd = find(charValueSolLog);
        charSpeedSolLog = logical(strcmp(charSpeed,solD.colheaders)); charSpeedSolInd = find(charSpeedSolLog);
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_rot.min(taskInd)),deg2rad(taskBounds.shoulder_rot.max(taskInd))),...
            MocoInitialBounds(solD.data(end,charValueSolInd)),MocoFinalBounds(0));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(solD.data(end,charSpeedSolInd)),MocoFinalBounds(0));
        clear mn mx charValue charSpeed charValueSolLog charValueSolInd charSpeedSolLog charSpeedSolInd

        %Elevation plane
        charValue = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/speed'];
        charValueSolLog = logical(strcmp(charValue,solD.colheaders)); charValueSolInd = find(charValueSolLog);
        charSpeedSolLog = logical(strcmp(charSpeed,solD.colheaders)); charSpeedSolInd = find(charSpeedSolLog);
        problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.elv_angle.min(taskInd)),deg2rad(taskBounds.elv_angle.max(taskInd))),...
            MocoInitialBounds(solD.data(end,charValueSolInd)),...
            MocoFinalBounds(deg2rad(taskBounds.elv_angle.ecc_LB(taskInd)),deg2rad(taskBounds.elv_angle.ecc_UB(taskInd))));
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(solD.data(end,charSpeedSolInd)),MocoFinalBounds(0));
        clear mn mx charValue charSpeed charValueSolLog charValueSolInd charSpeedSolLog charSpeedSolInd

        %Elbow flexion
        charValue = ['/jointset/',char(coordSet.get('elbow_flexion').getJoint().getName()),'/',char(coordSet.get('elbow_flexion').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('elbow_flexion').getJoint().getName()),'/',char(coordSet.get('elbow_flexion').getName()),'/speed'];
        charValueSolLog = logical(strcmp(charValue,solD.colheaders)); charValueSolInd = find(charValueSolLog);
        charSpeedSolLog = logical(strcmp(charSpeed,solD.colheaders)); charSpeedSolInd = find(charSpeedSolLog);
        mn = coordSet.get('elbow_flexion').getRangeMin();
        if contains(taskName,'Reach')
            %Limit elbow flexion to 90 degrees
            mx = deg2rad(90);
        else
            %Leave as max attainable elbow flexion
            mx = coordSet.get('elbow_flexion').getRangeMax();
        end
        problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(solD.data(end,charValueSolInd)),MocoFinalBounds(0))
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(solD.data(end,charSpeedSolInd)),MocoFinalBounds(0));
        clear mn mx charValue charSpeed charValueSolLog charValueSolInd charSpeedSolLog charSpeedSolInd

        %Forearm
        charValue = ['/jointset/',char(coordSet.get('pro_sup').getJoint().getName()),'/',char(coordSet.get('pro_sup').getName()),'/value'];
        charSpeed = ['/jointset/',char(coordSet.get('pro_sup').getJoint().getName()),'/',char(coordSet.get('pro_sup').getName()),'/speed'];
        charValueSolLog = logical(strcmp(charValue,solD.colheaders)); charValueSolInd = find(charValueSolLog);
        charSpeedSolLog = logical(strcmp(charSpeed,solD.colheaders)); charSpeedSolInd = find(charSpeedSolLog);
        mn = coordSet.get('pro_sup').getRangeMin();
        mx = coordSet.get('pro_sup').getRangeMax();
        problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(solD.data(end,charValueSolInd)),MocoFinalBounds(0))
        problem.setStateInfo(charSpeed,MocoBounds(-50,50),MocoInitialBounds(solD.data(end,charSpeedSolInd)),MocoFinalBounds(0));
        clear mn mx charValue charSpeed charValueSolLog charValueSolInd charSpeedSolLog charSpeedSolInd
        
        %Return to output directory
        cd(outputFolder);

    end
    
    %% Set-up muscle parameters

    if muscleDriven
        %Set the muscle activation state bounds
        %Set initial activation to be zero
        for m = 0:osimModel.getMuscles().getSize()-1
            %Get current muscle name
            muscName = osimModel.updMuscles().get(m).getName();
            %Create string for setting state info
            stateStr = ['/forceset/',char(muscName),'/activation'];
            %Set activation bounds. Include a specification that initial activation
            %must be zero for the majority of movements, with the exception of the
            %abduction 90-180 and abduction/external rotation 0-90 movements - as
            %these do not start in a neutral position
            if strcmpi(taskName,'Abd90_180') || strcmpi(taskName,'AbdExtRot0_90')
                %Set the activation bounds
                problem.setStateInfo(stateStr,MocoBounds(0,1)); 
            else
                %Set the activation bounds
                problem.setStateInfo(stateStr,MocoBounds(0,1),MocoInitialBounds(0));
            end        
            %Cleanup
            clear muscName stateStr fibStr
        end
        clear m

        %Replace the muscles in the model with muscles from DeGroote, Fregly, 
        %et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem 
        %Formulations for Solving the Muscle Redundancy Problem". These muscles
        %have the same properties as the original muscles but their characteristic
        %curves are optimized for direct collocation (i.e. no discontinuities, 
        %twice differentiable, etc).
        DeGrooteFregly2016Muscle().replaceMuscles(osimModel);

        %Turn off muscle-tendon dynamics to keep the problem simple.
        %This is probably already done in the model anyway
        for m = 0:osimModel.getMuscles().getSize()-1
            musc = osimModel.updMuscles().get(m);
            musc.set_ignore_tendon_compliance(true);
        end
        clear m
    else
        %no need to set muscle parameters
    end

    %% Finish setting up problem

    %Update problem
    problem = moco.updProblem();
    
    %Finalize model connections
    osimModel.finalizeConnections();

    %Set model to the moco problem
    problem.setModelCopy(osimModel);

    %% Set cost functions for task

    %Create marker end point costs for concentric tasks
    %Otherwise the final bounds for the relevant joint angles have been set
    
    if contains(taskName,'Concentric')

        switch taskName

            case 'ConcentricAxillaTouch'

                %Get the desired end point of the movement. This will involve the
                %marker on the middle finger reaching a point that represents the
                %acromion on the left side of the body

                %Get the position of the Acrom and IJ marker in the ground
                osimModel_state = osimModel.initSystem();
                acrom = osimModel.getMarkerSet().get('Acrom').getLocationInGround(osimModel_state);
                IJ = osimModel.getMarkerSet().get('IJ').getLocationInGround(osimModel_state);

                %Calculate the distance along the +Z axis from IJ to Acrom marker
                acrom_IJ_dist = acrom.get(2) - IJ.get(2);

                %Prescribe the marker end point using the X and Y position of the
                %Acrom marker and the distance from the IJ to Acrom along the -Z
                %axis for the Z position
                axillaReachPoint = Vec3(acrom.get(0),acrom.get(1),IJ.get(2) - acrom_IJ_dist);

                %Cleanup
                clear acrom IJ acrom_IJ_dist

                %Add the end point costs with appropriate weights
                endPointCost1 = MocoMarkerEndpointCost('MF_endPoint',5);
                endPointCost1.setPointName('/markerset/MiddleFinger');
                endPointCost1.setReferenceLocation(axillaReachPoint);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);

            case 'ConcentricForwardReach'

                %Get the desired end point of the movement. This will be at a the
                %level of the RibL marker at a distance 200% of forearm length in
                %front of the shoulder joint centre

                %Get the position of the shoulder joint centre. Note that the 1 corresponds
                %to the humphant_offset frame. This command also transforms it to the
                %ground frame.
                osimModel_state = osimModel.initSystem();
                SJC_ground = osimModel.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(osimModel_state);

                %Calculate the distance of the forearm (i.e. between the elbow and wrist
                %joint centre).

                %Get the position of the joint centres. Joint 1 corresponds to ulna offset
                %frame for elbow and joint 0 the radius offset for the radius hand joint
                EJC_ground = osimModel.getJointSet().get('elbow').get_frames(1).getPositionInGround(osimModel_state);
                WJC_ground = osimModel.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(osimModel_state);

                %Calculate the distance between the joint centres
                elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                FA_length = dist_markers(elbow,wrist);
                clear elbow wrist

                %Get the position of the RibL marker in the ground
                RibL = osimModel.getMarkerSet().get('RibL').getLocationInGround(osimModel_state);

                %Calculate the position two forearm length in front of the shoulder. In
                %front is represented by positive X
                forwardReachPoint = [SJC_ground.get(0)+(FA_length*2),RibL.get(1),SJC_ground.get(2)];

                %Create a marker end point cost for the reach position. Need to use the
                %markers on both sides of the wrist and the top of the hand to ensure that
                %the hand is placed level and palmar side down at the end - as such, need
                %to create markers end points for each of these.

                %Identify the distance between the two wrist markers
                RS = osimModel.getMarkerSet().get('RS').getLocationInGround(osimModel_state);
                US = osimModel.getMarkerSet().get('US').getLocationInGround(osimModel_state);
                RS = [RS.get(0),RS.get(1),RS.get(2)];
                US = [US.get(0),US.get(1),US.get(2)];
                wristWidth = dist_markers(RS,US);

                %Add and subtract half of the wrist distance from the original marker end
                %point along the Z-axis to get the proposed end points for the markers. It
                %is positive Z in the ground frame for the ulna marker and negative Z for
                %the radius marker
                US_endLoc = Vec3(forwardReachPoint(1),forwardReachPoint(2),forwardReachPoint(3)+(wristWidth/2));
                RS_endLoc = Vec3(forwardReachPoint(1),forwardReachPoint(2),forwardReachPoint(3)-(wristWidth/2));

                %Measure the distance from the wrist joint centre to the wri_out marker for
                %prescribing where the hand needs to go.
                wri_out = osimModel.getMarkerSet().get('wri_out').getLocationInGround(osimModel_state);
                wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                wristHeight = dist_markers(wri_out,wrist);

                %Add the wirst height amount along the y-axis from the proposed reach point
                %to get the point where the wri_out marker needs to go
                W_endLoc = Vec3(forwardReachPoint(1),forwardReachPoint(2)+wristHeight,forwardReachPoint(3));

                %Add the end point costs equally weighted
                endPointCost1 = MocoMarkerEndpointCost('RS_endPoint',5);
                endPointCost1.setPointName('/markerset/RS');
                endPointCost1.setReferenceLocation(RS_endLoc);
                endPointCost2 = MocoMarkerEndpointCost('US_endPoint',5);
                endPointCost2.setPointName('/markerset/US');
                endPointCost2.setReferenceLocation(US_endLoc);
                endPointCost3 = MocoMarkerEndpointCost('W_endPoint',5);
                endPointCost3.setPointName('/markerset/wri_out');
                endPointCost3.setReferenceLocation(W_endLoc);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);
                problem.addCost(endPointCost2);
                problem.addCost(endPointCost3);

            case 'ConcentricHairTouch'

                %Get the desired end point of the movement. This will be at an
                %arbitrary point above the C7 marker (0.25m looks good for now)
                %where the middle finger needs to reach

                %Get the position of the C7 marker
                osimModel_state = osimModel.initSystem();
                C7 = osimModel.getMarkerSet().get('C7').getLocationInGround(osimModel_state);

                %Prescribe the marker end point using the X and Z coordinates of
                %the C7 marker and add the arbitrary distance to the Y position
                hairReachPoint = Vec3(C7.get(0),C7.get(1) + 0.25,C7.get(2));

                %Cleanup
                clear C7

                %Add the end point costs with appropriate weights
                endPointCost1 = MocoMarkerEndpointCost('MF_endPoint',5);
                endPointCost1.setPointName('/markerset/MiddleFinger');
                endPointCost1.setReferenceLocation(hairReachPoint);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);

            case 'ConcentricRearTouch'

                %Get the desired end point of the movement. This will e at the X
                %and Z position of the C7 marker and at the Y level of the RibL
                %marker

                %Get the position of the C7 and RibL markers in the ground frame
                osimModel_state = osimModel.initSystem();
                C7 = osimModel.getMarkerSet().get('C7').getLocationInGround(osimModel_state);
                RibL = osimModel.getMarkerSet().get('RibL').getLocationInGround(osimModel_state);

                %Prescribe the marker end point using these marker positions
                rearReachPoint = Vec3(C7.get(0),RibL.get(1),C7.get(2));

                %Cleanup
                clear C7 RibL

                %Add the end point costs with appropriate weights
                endPointCost1 = MocoMarkerEndpointCost('MF_endPoint',5);
                endPointCost1.setPointName('/markerset/MiddleFinger');
                endPointCost1.setReferenceLocation(rearReachPoint);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);

            case 'ConcentricUpwardReach90'

                %Get the desired end point of the movement. This will be at a point 15
                %degrees above the shoulder at a distance of 200% of forearm length.
                %(note there is no prescribed distance in the Vidt paper)

                %Get the position of the shoulder joint centre. Note that the 1 corresponds
                %to the humphant_offset frame. This command also transforms it to the
                %ground frame.
                osimModel_state = osimModel.initSystem();
                SJC_ground = osimModel.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(osimModel_state);

                %Calculate the distance of the forearm (i.e. between the elbow and wrist
                %joint centre).

                %Get the position of the joint centres. Joint 1 corresponds to ulna offset
                %frame for elbow and joint 0 the radius offset for the radius hand joint
                EJC_ground = osimModel.getJointSet().get('elbow').get_frames(1).getPositionInGround(osimModel_state);
                WJC_ground = osimModel.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(osimModel_state);

                %Calculate the distance between the joint centres
                elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                FA_length = dist_markers(elbow,wrist);
                clear elbow wrist

                %Calculate the position two forearm length in front of the shoulder. In
                %front is represented by positive X
                upwardReachPoint = [SJC_ground.get(0)+(FA_length*2),SJC_ground.get(1),SJC_ground.get(2)];

                %Cleanup
                clear Xdist theta Ydist

                %Create a marker end point cost for the reach position. Need to use the
                %markers on both sides of the wrist and the top of the hand to ensure that
                %the hand is placed level and palmar side down at the end - as such, need
                %to create markers end points for each of these.

                %Identify the distance between the two wrist markers
                osimModel_state = osimModel.initSystem();
                RS = osimModel.getMarkerSet().get('RS').getLocationInGround(osimModel_state);
                US = osimModel.getMarkerSet().get('US').getLocationInGround(osimModel_state);
                RS = [RS.get(0),RS.get(1),RS.get(2)];
                US = [US.get(0),US.get(1),US.get(2)];
                wristWidth = dist_markers(RS,US);

                %Add and subtract half of the wrist distance from the original marker end
                %point along the Z-axis to get the proposed end points for the markers. It
                %is positive Z in the ground frame for the ulna marker and negative Z for
                %the radius marker
                US_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)+(wristWidth/2));
                RS_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)-(wristWidth/2));

                %Measure the distance from the wrist joint centre to the wri_out marker for
                %prescribing where the hand needs to go.
                wri_out = osimModel.getMarkerSet().get('wri_out').getLocationInGround(osimModel_state);
                wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                wristHeight = dist_markers(wri_out,wrist);

                %Add the wirst height amount along the y-axis from the proposed reach point
                %to get the point where the wri_out marker needs to go
                W_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2)+wristHeight,upwardReachPoint(3));

                %Add the end point costs equally weighted to contribute 50% to the problem
                endPointCost1 = MocoMarkerEndpointCost('RS_endPoint',5);
                endPointCost1.setPointName('/markerset/RS');
                endPointCost1.setReferenceLocation(RS_endLoc);
                endPointCost2 = MocoMarkerEndpointCost('US_endPoint',5);
                endPointCost2.setPointName('/markerset/US');
                endPointCost2.setReferenceLocation(US_endLoc);
                endPointCost3 = MocoMarkerEndpointCost('W_endPoint',5);
                endPointCost3.setPointName('/markerset/wri_out');
                endPointCost3.setReferenceLocation(W_endLoc);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);
                problem.addCost(endPointCost2);
                problem.addCost(endPointCost3);

            case 'ConcentricUpwardReach105'

                %Get the desired end point of the movement. This will be at a point 15
                %degrees above the shoulder at a distance of 200% of forearm length.
                %(note there is no prescribed distance in the Vidt paper)

                %Get the position of the shoulder joint centre. Note that the 1 corresponds
                %to the humphant_offset frame. This command also transforms it to the
                %ground frame.
                osimModel_state = osimModel.initSystem();
                SJC_ground = osimModel.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(osimModel_state);

                %Calculate the distance of the forearm (i.e. between the elbow and wrist
                %joint centre).

                %Get the position of the joint centres. Joint 1 corresponds to ulna offset
                %frame for elbow and joint 0 the radius offset for the radius hand joint
                EJC_ground = osimModel.getJointSet().get('elbow').get_frames(1).getPositionInGround(osimModel_state);
                WJC_ground = osimModel.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(osimModel_state);

                %Calculate the distance between the joint centres
                elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                FA_length = dist_markers(elbow,wrist);
                clear elbow wrist

                %Calculate the position 200% forearm length in front of the shoulder. In
                %front is represented by positive X
                inFrontPoint = [SJC_ground.get(0)+(FA_length*2),SJC_ground.get(1),SJC_ground.get(2)];

                %Calculate how far above this point is needed to generate a 15 degree angle
                %above the level of the shoulder joint. Calculate this using a 2D triangle
                %encompassing the X and Y axis

                %Calculate horizontal distance from shoulder to in front point
                Xdist = (FA_length*2) - SJC_ground.get(0);
                %Set angle to calculate height with
                theta = 15;
                %Calculate height of triangle
                Ydist = tand(15) * Xdist;

                %Prescribe upward reach point
                upwardReachPoint = [inFrontPoint(1),inFrontPoint(2)+Ydist,inFrontPoint(3)];

                %Cleanup
                clear Xdist theta Ydist

                %Create a marker end point cost for the reach position. Need to use the
                %markers on both sides of the wrist and the top of the hand to ensure that
                %the hand is placed level and palmar side down at the end - as such, need
                %to create markers end points for each of these.

                %Identify the distance between the two wrist markers
                osimModel_state = osimModel.initSystem();
                RS = osimModel.getMarkerSet().get('RS').getLocationInGround(osimModel_state);
                US = osimModel.getMarkerSet().get('US').getLocationInGround(osimModel_state);
                RS = [RS.get(0),RS.get(1),RS.get(2)];
                US = [US.get(0),US.get(1),US.get(2)];
                wristWidth = dist_markers(RS,US);

                %Add and subtract half of the wrist distance from the original marker end
                %point along the Z-axis to get the proposed end points for the markers. It
                %is positive Z in the ground frame for the ulna marker and negative Z for
                %the radius marker
                US_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)+(wristWidth/2));
                RS_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)-(wristWidth/2));

                %Measure the distance from the wrist joint centre to the wri_out marker for
                %prescribing where the hand needs to go.
                wri_out = osimModel.getMarkerSet().get('wri_out').getLocationInGround(osimModel_state);
                wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
                wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
                wristHeight = dist_markers(wri_out,wrist);

                %Add the wirst height amount along the y-axis from the proposed reach point
                %to get the point where the wri_out marker needs to go
                W_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2)+wristHeight,upwardReachPoint(3));

                %Add the end point costs equally weighted to contribute 50% to the problem
                endPointCost1 = MocoMarkerEndpointCost('RS_endPoint',5);
                endPointCost1.setPointName('/markerset/RS');
                endPointCost1.setReferenceLocation(RS_endLoc);
                endPointCost2 = MocoMarkerEndpointCost('US_endPoint',5);
                endPointCost2.setPointName('/markerset/US');
                endPointCost2.setReferenceLocation(US_endLoc);
                endPointCost3 = MocoMarkerEndpointCost('W_endPoint',5);
                endPointCost3.setPointName('/markerset/wri_out');
                endPointCost3.setReferenceLocation(W_endLoc);

                %Add the end point cost along with an effort cost.
                problem.addCost(endPointCost1);
                problem.addCost(endPointCost2);
                problem.addCost(endPointCost3);

        end
        
    else
        %no marker costs need to be added
    end

    %Add the effort cost with specific weighting
    problem.addCost(MocoControlCost('effort',1));

    %Add the final time cost for relevant movements. This does not apply to the
    %simplistic movements which have a set end time bount
    if strcmpi(taskName,'Abd0_90') || strcmpi(taskName,'Abd90_180') || ...
            strcmpi(taskName,'ExtRot0_90') || strcmpi(taskName,'AbdExtRot0_90')
        %don't add final time cost
    else
        problem.addCost(MocoFinalTimeCost('time',1));
    end

    %% Print tool to file

    %Set whether muscle or torque driven
    if muscleDriven
        simType = 'MuscleDriven';
    else
        simType = 'TorqueDriven';
    end

    cd(outputFolder);
    moco.setName([modelName,'_',taskName,'_',simType,'_',num2str(nMeshPoints),'meshPoints']);
    moco.print([modelName,'_',taskName,'_',simType,'_',num2str(nMeshPoints),'meshPoints.omoco']);

    %% Update solver

    %Update the underlying MocoCasADiSolver with the new problem.
    solver = MocoCasADiSolver.safeDownCast(moco.updSolver());
    solver.resetProblem(problem);

    %Set guess
    if ~exist('guessFile','var') || isempty(guessFile)
        solver.setGuess('bounds'); % This is also the default setting.
    else
        solver.setGuessFile(guessFile);
    end

    %% Run predictive problem
    
    %Setup diary to record solver output
    clc;
    diary IPOPT.log
    
    %Start timer
    tic
    
    %Run optimisation
    disp('Beginning optimisation...');
    solution = moco.solve();
    runtime = toc;
    if muscleDriven
        disp([modelName,' ',taskName,' muscle driven with ',num2str(nMeshPoints),' mesh points solver complete. Total time = ', num2str(runtime/60),' minutes.']);
    else
        disp([modelName,' ',taskName,' torque driven with ',num2str(nMeshPoints),' mesh points solver complete. Total time = ', num2str(runtime/60),' minutes.']);
    end
    %Print details to text file
    fid = fopen([modelName,'_',taskName,'_',simType,'_',num2str(nMeshPoints),'meshPoints_Details.txt'],'wt');
    if muscleDriven
        fprintf(fid,[modelName,' ',taskName,' muscle driven with ',num2str(nMeshPoints),' mesh points solver complete. Total time = ', num2str(runtime/60),' minutes.']);
    else
        fprintf(fid,[modelName,' ',taskName,' torque with ',num2str(nMeshPoints),' mesh points solver complete. Total time = ', num2str(runtime/60),' minutes.']);
    end
    fclose(fid);
    
    %Turn off diary
    diary off
    
    %Rename diary
    movefile('IPOPT.log',['IPOPToutput_',taskName,'_',num2str(nMeshPoints),'meshPoints.log']);
    
end

end









