function ShoulderStrengthSims_1_GenerateModels(baselineModel,scaleFactors)
    
    %Function to generate models with altered strength parameters in
    %specific muscle groups by the factors presented uin the
    %scaleFactors variable. This is a list of integers in decimals that
    %represent the scale factor to apply to muscle strengths.
    %
    %INPUTS:    baselineModel = full string path to original/baseline model to use
    %           scaleFactors = set of integers to scale strength by (e.g. 0.9,1.1)
    %
    %Muscle groups that are edited:
        %scapula_protractors = all parts of serratus anterior and pec minor
        %external_rotators = teres minor, infraspinatus & post. deltoid
        %scapula_retractors = all rhomboid parts, upper & middle trapezius
        %abductors = all parts of deltoid & supraspinatus
            %also includes upper & lower trapezius, serratus anterior as upward scapula rotators 
        %adductors  = lattisimus dorsi, lower pec major, teres major, subscapularis, coracobrachialis
            %also includes levator scapulae, all parts of rhomboids and pec minor as downward scapula rotators 
        %internal_rotators = subscapularis, latissimus dorsi, teres major, pec major
        %flexors = superior pec major, anterior deltoid, coracobrachialis
            %also includes upper & lower trapezius, serratus anterior as upward scapula rotators 
        %horizontal_adductors = pec major, anterior deltoid
            %also includes all parts of serratus anterior and pec minor as scapula protractors
    
    import org.opensim.modeling.*

    %Get the output directory based on where the model is
    outputDir = fileparts(baselineModel);
    
    %Create strings for the muscle groups to alter
    muscleGroups = [{'scapula_protractors'};
        {'external_rotators'};
        {'scapula_retractors'};
        {'abductors'};
        {'internal_rotators'};
        {'flexors'};
        {'horizontal_adductors'}];
    
    %Set the muscles within each group
    muscleLists.scapula_protractors = [{'SRA1'}; {'SRA2'}; {'SRA3'}; {'PMN'}];
    muscleLists.external_rotators = [{'TMIN'}; {'INFSP'}; {'DELT3'}];
    muscleLists.scapula_retractors = [{'RMN'}; {'RMJ1'}; {'RMJ2'}; {'TRP1'}; {'TRP2'}; {'TRP3'}];
    muscleLists.abductors = [{'DELT1'}; {'DELT2'}; {'DELT3'}; {'SUPSP'}; {'TRP1'}; {'TRP4'}; {'SRA1'}; {'SRA2'}; {'SRA3'}];
    muscleLists.adductors = [{'LAT'}; {'PECM3'}; {'TMAJ'}; {'SUBSC'}; {'CORB'}; {'LVS'}; {'RMN'}; {'RMJ1'}; {'RMJ2'}; {'PMN'}];
    muscleLists.internal_rotators = [{'SUBSC'}; {'LAT'}; {'TMAJ'}; {'PECM1'}; {'PECM2'}; {'PECM3'}];
    muscleLists.flexors = [{'PECM1'}; {'DELT1'}; {'CORB'}; {'TRP1'}; {'TRP4'}; {'SRA1'}; {'SRA2'}; {'SRA3'}];
    muscleLists.horizontal_adductors = [{'PECM1'}; {'PECM2'}; {'PECM3'}; {'DELT1'}; {'SRA1'}; {'SRA2'}; {'SRA3'}; {'PMN'}];
    
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
            copyModel.print([outputDir,'\',muscleGroups{m},'_strength',num2str(scaleFactors(s)*100),'.osim']);
            %Cleanup
            clear copyModel
        end
        clear s        
    end
    clear m
    
    %%%--- End of ShoulderStrengthSims_1_GenerateModels.m ---%%%

end