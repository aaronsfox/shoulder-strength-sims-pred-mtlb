function ShoulderStrengthSims_2_RunSimulations(taskName,meshInterval)
    
    % @author: Aaron Fox
    % Centre for Sport Research, Deakin University
    % aaron.f@deakin.edu.au
    %
    % Function to run the predictive simulations for models with different
    % muscle strengths. The function takes the input task name into the
    % problem solver script and runs the predictive simulations with the
    % models of varying strength identified in the models folder
    %
    % INPUTS:
    %
    % taskName: string containing task to be simulated. Can be one of 
    % 'ConcentricUpwardReach105'; 'ConcentricForwardReach' or 'HairTouch'
    %
    % meshInterval: desired mesh interval to use in running predictive
    % simulations. Typically chosen from the outputs of the previous function
    % (i.e. 'ShoulderCapsulorrhaphySims_4_NodeSelection').
    
    import org.opensim.modeling.*
    warning off
    home_dir = pwd;
    
    %Run checks
    if nargin < 1
        %Check for task
        taskNo = input('Select desired task:\n[1] Upward Reach 105\n[2] Forward Reach\n[3] Hair Touch\nEnter number: ');
        switch taskNo
            case 1
                taskName = 'ConcentricUpwardReach105';
            case 2
                taskName = 'ConcentricForwardReach';
            case 3
                taskName = 'HairTouch';
        end        
    end
    
    
    if nargin < 2
        %Set default mesh intervals based on node selection process
        if strcmp(taskName,'ConcentricUpwardReach105')
            meshInterval = 100;            
        elseif strcmp(taskName,'ConcentricForwardReach')
            meshInterval = 100;            
        elseif strcmp(taskName,'HairTouch')
            meshInterval = 75;            
        end
    end
    
    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));
    
% % %     %Load the custom GH JRF path constraint plugin
% % %     cd('..\..\Plugin_Moco_GHJRF_Constraint\build\Release');
% % %     opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimMocoGHJRFConstraint.dll']);
%%%%%%% plugin seems to be causing normal Moco commands to fail
    
    %Create space to store results
    cd('..\..');
    mkdir('SimulationResults'); cd('SimulationResults');
    mkdir(taskName); cd(taskName); taskDir = [pwd,'\'];
    
    
    %% Identify the models to iterate through for predictive simulations
    
    %Go to model directory
    cd('..\..\Models');
    modelDir = [pwd,'\'];
    %Add geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths([modelDir,'Geometry']);
    
    %Grab the model names
    f = dir('*.osim');
    for ff = 1:length(f)
        modelFiles{ff,1} = [modelDir,f(ff).name];
        modelName{ff,1} = f(ff).name(1:end-5);
    end
    clear ff f
    
    %% Run simulations
    
    %Load the task bounds data
    cd('..\SupportingData');
    taskBounds = load('Vidt_MovementValues.mat');
    
    %Set the guess file for the current task
    %Note that these come from an existing study
    %%%%% TODO: add reference to existing work
    cd('..\GuessFiles');
    guessFile = [pwd,'\guessFile_',taskName,'.sto'];
    
    %return to task directory
    cd(taskDir);
    
    %% Loop through models
    for ii = 1:length(modelFiles)
        
        %Set up the Moco study
        study = MocoStudy();

        %Initialise the problem
        problem = study.updProblem();

        %Load model (function uses the starting 'None' model)
        osimModel = Model(modelFiles{ii});

        %Lock the thorax joints of the model to make this a shoulder only movement
        coordSet = osimModel.updCoordinateSet();
        coordSet.get('thorax_tilt').set_locked(true);
        coordSet.get('thorax_list').set_locked(true);
        coordSet.get('thorax_rotation').set_locked(true);
        coordSet.get('thorax_tx').set_locked(true);
        coordSet.get('thorax_ty').set_locked(true);
        coordSet.get('thorax_tz').set_locked(true);

        %Add a 1kg mass for the reaching tasks
        if contains(taskName,'Reach')
            %Get hand mass and calculate added 1kg value
            newHandMass = osimModel.getBodySet().get('hand_r').getMass() + 1;
            %Set new hand mass
            osimModel.getBodySet().get('hand_r').setMass(newHandMass);
            %Cleanup
            clear newHandMass
        end

        %Add relevant torque actuators to each degree of freedom.
        %Loop through and add coordinate actuators
        %Don't add anything for the thorax
        for c = 1:coordSet.getSize()
            if contains(char(coordSet.get(c-1).getName()),'thorax')
                %don't add a coordinate actuator as these coordinates won't move    
            elseif strcmp(char(coordSet.get(c-1).getName()),'elbow_flexion')
                %Add an idealised torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),75,[inf,-inf],'_torque');
            elseif strcmp(char(coordSet.get(c-1).getName()),'pro_sup')
                %Add an idealised torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),30,[inf,-inf],'_torque');
            elseif strcmp(char(coordSet.get(c-1).getName()),'elv_angle')
                %Add a reserve torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),1,[inf,-inf],'_reserve');
            elseif strcmp(char(coordSet.get(c-1).getName()),'shoulder_elv') || ...
                    strcmp(char(coordSet.get(c-1).getName()),'shoulder_rot')
                %Add a reserve torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),1,[1,-1],'_reserve');
            end
        end
        clear c

        %Finalize model connections
        osimModel.finalizeConnections();

        %Set model in problem
        problem.setModel(osimModel);

        %Set time bounds on the problem
        problem.setTimeBounds(0, [0.1,1.0]);

        %Add state bounds relevant to task
        addTaskBounds(taskName,taskBounds,problem,osimModel);

        %Create marker end point costs for task    
        addMarkerFinalGoals(taskName,problem,osimModel);

        %Add a MocoControlCost to the problem.
        problem.addGoal(MocoControlGoal('effort',1));

        %Add a MocoFinalTimeGoal to the problem
        problem.addGoal(MocoFinalTimeGoal('time',1));
        
        %Add a joint reaction force goal to the problem. The aim of this
        %goal is to minimise the shear forces during the task.
        jrfGoal = MocoJointReactionGoal('jrfGoal',1);
        jrfGoal.setJointPath('/jointset/unrothum');
        jrfGoal.setLoadsFrame('parent');
        jrfGoal.setExpressedInFramePath('/jointset/unrothum/rotated_scap_frame');
        reactionMeasure = StdVectorString();
        reactionMeasure.add('force-x'); reactionMeasure.add('force-z');
        jrfGoal.setReactionMeasures(reactionMeasure);
        problem.addGoal(jrfGoal);

        %Configure the solver.
        solver = study.initCasADiSolver();
        solver.set_num_mesh_intervals(meshInterval);
        solver.set_verbosity(2);
        solver.set_optim_solver('ipopt');
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-4);
        
        %Set the guess to the relevant solution

        %Some solution files seem to generate nan's in the last row
        %of the slack variables which generates a Casadi error when
        %attempting to use as a guess. To resolve this, a random
        %guess can be created (which seems to set the slacks to
        %zero), and be filled with the relevant data 9states,
        %controls, multipliers) from the existing solution.

        %Grab the last solution
        mocoTraj = MocoTrajectory(guessFile);

        %Create random guessusing the current solver
        randTraj = solver.createGuess();
        
        %Set time for random guess to original trajectory
        randTraj.setTime(mocoTraj.getTime())

        %Refresh states in random guess from the moco trajectory
        randTraj.setStatesTrajectory(mocoTraj.exportToStatesTable());

        %Refresh controls from moco trajectory
        controlNames = mocoTraj.getControlNames();
        for cc = 0:controlNames.size()-1
            randTraj.setControl(controlNames.get(cc),mocoTraj.getControlMat(controlNames.get(cc)))    
        end
        clear cc controlNames

        %Refresh multipliers from moco trajectory
        multiplierNames = mocoTraj.getMultiplierNames();
        for cc = 0:multiplierNames.size()-1
            randTraj.setMultiplier(multiplierNames.get(cc),mocoTraj.getMultiplierMat(multiplierNames.get(cc)))    
        end
        clear cc multiplierNames

        %Set the guess to this newly created trajectory
        solver.setGuess(randTraj); clear mocoTraj
              
        %Solve
        study.setName([modelName{ii},'_',taskName,'_',num2str(meshInterval*2+1),'nodes']);
        clc
        predictSolution = study.solve();
        
        %%%%% TO DO: still need something in place for unsealing if
        %%%%% solution failes...       
        
    end
    clear ii
    
    
    
    
    
    
    %%

    %%%--- End of ShoulderStrengthSims_2_RunSimulations.m ---%%%
    
end