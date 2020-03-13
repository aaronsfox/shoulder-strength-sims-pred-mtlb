function ShoulderStrengthSims_2_RunSimulations(taskName,modelDir,resultsDir,taskBounds,whichModels)
    
    %Function to run the predictive simulations for models with different
    %muscle strengths. The function takes the input task name into the
    %problem solver script and runs the predictive simulations with the
    %models of varying strength identified in the models folder
    %
    %INPUTS:    taskName = string containing appropriate task name
        % Current options are:
                % ConcentricAxillaTouch
                % ConcentricForwardReach
                % ConcentricHairTouch
                % ConcentricRearTouch
                % ConcentricUpwardReach90
                % ConcentricUpwardReach105

    %           modelDir = full path to directory containing models 
    %           resultsDir = full path to location for saving simulation results 
    %           taskBounds = structure containing task bound values from Vidt et al. data to pass to solver
    %           whichModels = string of 'all' 'weakened' or 'strengthened' of which set of muscles to run
    
    import org.opensim.modeling.*
    warning off
    home_dir = cd;
    
    %%%%% TO DO: checks for inputs...
    
    %% Identify the models to iterate through for predictive simulations
    
    %Check for trailing slash in model directory
    if strcmp(modelDir(end),'\') || strcmp(modelDir(end),'/')
        %trailing slash present
    else
        %add the trailing slash
        modelDir = [modelDir,'\'];
    end
    
    %Navigate to model directory
    cd(modelDir);
    
    %identify files with .osim extension
    f = dir('*.osim');
    %grab out the number for each filename representing strength value
    for ff = 1:length(f)
        S = sprintf('%s ', f(ff).name(:));
        letters = (S < '0' | S > '9');  % Mask all non-numbers
        S(letters) = ' ';
        strengthVal(ff,1) = str2double(S);
        clear S letters
    end
    
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
            %Allocate model files
            for ff = 1:length(f)
                modelNames{ff,1} = f(ff).name;
            end
            clear ff f
            modelFiles = modelNames(grabModel);
            
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
            %Allocate model files
            for ff = 1:length(f)
                modelNames{ff,1} = f(ff).name;
            end
            clear ff f
            modelFiles = modelNames(grabModel);
            
        case 'all'
            %grab all models
            for ff = 1:length(f)
                modelFiles{ff,1} = f(ff).name;
            end
            clear ff f
            
    end
    
    %% Run iterations of predictive simulations. This includes three iterations
    %  which increase the number of mesh points each time
    
    %Before running simulations create a directory to store the results of
    %the current movement task in
    cd(resultsDir);
    mkdir(taskName); cd(taskName);
    %Set output folder to save results
    baseDir = pwd;
    
    %Loop through the different models as an outer process    
    
    %Set if statement if it's a basic movement vs. functional movement
    if strcmp(taskName,'Abd0_90') || strcmp(taskName,'Abd90_180') || ...
            strcmp(taskName,'ExtRot0_90') || strcmp(taskName,'AbdExtRot0_90')        
        %Run the problem solver function only once
        
        %Loop through the different models as an outer process
        for m = 1:length(modelFiles)

            %Set model file for each iteration
            ModelFile = [modelDir,modelFiles{m}];

            %Create directory to store results
            mkdir(modelFiles{m}(1:end-5)); cd(modelFiles{m}(1:end-5));

            %Set the current directory to be the output folder
            outputFolder = pwd;
            
            %Setup and run the MoCo solver (inverse solver programmed
            %within this function). nMesh points serves as the mesh
            %interval value here
            nMeshPoints = 0.01; iterationNumber = 1;
            SetupRun_MoCo_ProblemSolver(ModelFile,taskName,nMeshPoints,iterationNumber,outputFolder,true);

            %Cleanup before skipping to next model
            clear ModelFile iterationNumber nMeshPoints outputFolder

            %Return to base directory
            cd(baseDir);

            
        end
        clear m
        
    else
        %Run the problem solver for chained, multiple iterations
        
        %Loop through the different models as an outer process
        for m = 1:length(modelFiles)

            %Set model file for each iteration
            ModelFile = [modelDir,modelFiles{m}];

            %Create directory to store results
            mkdir(modelFiles{m}(1:end-5)); cd(modelFiles{m}(1:end-5));

            %Set the current directory to be the output folder
            outputFolder = pwd;

            %Set-up and run first iteration at 10 meshPoints      
            nMeshPoints = 10;
            SetupRun_MoCo_ProblemSolver(ModelFile,taskName,taskBounds,nMeshPoints,outputFolder,true);

            %Set-up and run second iteration at 50 meshPoints
            nMeshPoints = 50;  %reset settings that change
            %Set guess file
            guessFile = [outputFolder,'\',modelFiles{m}(1:end-5),'_',taskName,'_MuscleDriven_10meshPoints_solution.sto'];
            SetupRun_MoCo_ProblemSolver(ModelFile,taskName,taskBounds,nMeshPoints,outputFolder,true,guessFile);

% % %             %Set-up and run iteration 3
% % %             iterationNumber = 3; nMeshPoints = 100;  %reset settings that change
% % %             %Set guess file
% % %             guessFile = [outputFolder,'\',modelFiles{m}(1:end-5),'_',taskName,'_MuscleDriven_iteration2_solution.sto'];
% % %             SetupRun_MoCo_ProblemSolver(ModelFile,taskName,taskBounds,nMeshPoints,iterationNumber,outputFolder,true,guessFile);

            %Cleanup before skipping to next model
            clear ModelFile nMeshPoints outputFolder

            %Return to base directory
            cd(baseDir);

        end
        clear m
               
    end
    
    %Return to home directory
    cd(home_dir);

    %%%--- End of ShoulderStrengthSims_2_RunSimulations.m ---%%%
    
end