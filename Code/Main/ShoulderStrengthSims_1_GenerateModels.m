function ShoulderStrengthSims_1_GenerateModels
    
    %Function to generate models with altered strength parameters in
    %specific muscle groups.
    %
    %Muscle groups that are edited [TODO - adjust these to reflect what is actually included]:
        %...
    
    import org.opensim.modeling.*
    
    %Create path to baseline model
    cd('..\..\Models');
    modelPath = [pwd,'\'];
    %Add geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);
    %Set baseline model
    baselineModel = [modelPath,'BaselineModel.osim'];

    %Set scale factors to weaken muscles
    scaleFactors = [0.8; 0.9];
        
    %Create strings for the muscle groups to alter
    muscleGroups = [{'externalRotators'};
        {'internalRotators'};
        {'elevators'};
        {'horizontalAdductors'}];
    
    %Set the muscles within each group
    muscleLists.externalRotators = [{'TMIN'}; {'INFSP'}; {'SRA3'}; {'DELT3'}];
    muscleLists.internalRotators = [{'SUBSC'}; {'TMAJ'}; {'DELT1'}];
    muscleLists.elevators = [{'DELT1'}; {'DELT1'}; {'DELT3'}; {'SUPSP'}; {'PECM1'}; {'CORB'}];
    muscleLists.horizontalAdductors = [{'PECM1'}; {'PECM2'}; {'PECM3'}];
    
    %Loop through muscle groups and adjust strength by the listed scale
    %factors for each set of muscles
    for m = 1:length(muscleGroups)           
        %Loop through scale factors
        for s = 1:length(scaleFactors) 
            %Load in the baseline model
            copyModel = Model(baselineModel);    
            %Loop through the muscles within the current group
            for k = 1:length(muscleLists.(muscleGroups{m}))                
                %Get the current muscles strength
                currMuscStrength = copyModel.getMuscles().get(muscleLists.(muscleGroups{m}){k}).get_max_isometric_force();                
                %Scale to its new force generating capacity
                newMuscStrength = currMuscStrength * scaleFactors(s);
                %Set the new muscle strength in the model
                copyModel.getMuscles().get(muscleLists.(muscleGroups{m}){k}).set_max_isometric_force(newMuscStrength);    
                %Cleanup
                clear currMuscStrength newMuscStrength                
            end
            clear k
            %Save the new model to the output directory
            copyModel.finalizeConnections();
            copyModel.print([modelPath,muscleGroups{m},'_strength',num2str(scaleFactors(s)*100),'.osim']);
            %Cleanup
            clear copyModel
        end
        clear s        
    end
    clear m
    
    %%%--- End of ShoulderStrengthSims_1_GenerateModels.m ---%%%

end