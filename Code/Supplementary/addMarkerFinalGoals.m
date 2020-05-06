function addMarkerFinalGoals(taskName,mocoProblem,modelObject)

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% Convenience function for adding marker final goals into a Moco problem
% relevant to the specific task
%
% Input:    taskName - string of the task name being simulated
%           mocoProblem - MocoProblem object to add goals to
%           modelObject - Opensim model object that is being used in problem

    import org.opensim.modeling.*

    %Check for values
    if nargin < 2
        %Throw error
        error('At least 2 inputs (a task name and Moco Problem) are required');
    end

    %% Add goals
    
    switch taskName
        
        
        case 'ConcentricUpwardReach105'
            
            %Get the desired end point of the movement. This will be at a point 15
            %degrees above the shoulder at a distance of 200% of forearm length.
            %(note there is no prescribed distance in the Vidt paper)

            %Get the position of the shoulder joint centre. Note that the 1 corresponds
            %to the humphant_offset frame. This command also transforms it to the
            %ground frame.
            modelObject_state = modelObject.initSystem();
            SJC_ground = modelObject.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(modelObject_state);

            %Calculate the distance of the forearm (i.e. between the elbow and wrist
            %joint centre).

            %Get the position of the joint centres. Joint 1 corresponds to ulna offset
            %frame for elbow and joint 0 the radius offset for the radius hand joint
            EJC_ground = modelObject.getJointSet().get('elbow').get_frames(1).getPositionInGround(modelObject_state);
            WJC_ground = modelObject.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(modelObject_state);

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
            modelObject_state = modelObject.initSystem();
            RS = modelObject.getMarkerSet().get('RS').getLocationInGround(modelObject_state);
            US = modelObject.getMarkerSet().get('US').getLocationInGround(modelObject_state);
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
            wri_out = modelObject.getMarkerSet().get('wri_out').getLocationInGround(modelObject_state);
            wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
            wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
            wristHeight = dist_markers(wri_out,wrist);

            %Add the wirst height amount along the y-axis from the proposed reach point
            %to get the point where the wri_out marker needs to go
            W_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2)+wristHeight,upwardReachPoint(3));

            %Add the end point costs equally weighted to contribute 50% to the problem
            endPointCost1 = MocoMarkerFinalGoal('RS_endPoint',5);
            endPointCost1.setPointName('/markerset/RS');
            endPointCost1.setReferenceLocation(RS_endLoc);
            endPointCost2 = MocoMarkerFinalGoal('US_endPoint',5);
            endPointCost2.setPointName('/markerset/US');
            endPointCost2.setReferenceLocation(US_endLoc);
            endPointCost3 = MocoMarkerFinalGoal('W_endPoint',5);
            endPointCost3.setPointName('/markerset/wri_out');
            endPointCost3.setReferenceLocation(W_endLoc);

            %Add the end point cost along with an effort cost.
            mocoProblem.addGoal(endPointCost1);
            mocoProblem.addGoal(endPointCost2);
            mocoProblem.addGoal(endPointCost3);
            
        
        case 'ConcentricForwardReach'
            
            %Get the desired end point of the movement. This will be at a the
            %level of the RibL marker at a distance 200% of forearm length in
            %front of the shoulder joint centre

            %Get the position of the shoulder joint centre. Note that the 1 corresponds
            %to the humphant_offset frame. This command also transforms it to the
            %ground frame.
            modelObject_state = modelObject.initSystem();
            SJC_ground = modelObject.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(modelObject_state);

            %Calculate the distance of the forearm (i.e. between the elbow and wrist
            %joint centre).

            %Get the position of the joint centres. Joint 1 corresponds to ulna offset
            %frame for elbow and joint 0 the radius offset for the radius hand joint
            EJC_ground = modelObject.getJointSet().get('elbow').get_frames(1).getPositionInGround(modelObject_state);
            WJC_ground = modelObject.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(modelObject_state);

            %Calculate the distance between the joint centres
            elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)];
            wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
            FA_length = dist_markers(elbow,wrist);
            clear elbow wrist

            %Get the position of the RibL marker in the ground
            RibL = modelObject.getMarkerSet().get('RibL').getLocationInGround(modelObject_state);

            %Calculate the position two forearm length in front of the shoulder. In
            %front is represented by positive X
            forwardReachPoint = [SJC_ground.get(0)+(FA_length*2),RibL.get(1),SJC_ground.get(2)];

            %Create a marker end point cost for the reach position. Need to use the
            %markers on both sides of the wrist and the top of the hand to ensure that
            %the hand is placed level and palmar side down at the end - as such, need
            %to create markers end points for each of these.

            %Identify the distance between the two wrist markers
            RS = modelObject.getMarkerSet().get('RS').getLocationInGround(modelObject_state);
            US = modelObject.getMarkerSet().get('US').getLocationInGround(modelObject_state);
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
            wri_out = modelObject.getMarkerSet().get('wri_out').getLocationInGround(modelObject_state);
            wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
            wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
            wristHeight = dist_markers(wri_out,wrist);

            %Add the wirst height amount along the y-axis from the proposed reach point
            %to get the point where the wri_out marker needs to go
            W_endLoc = Vec3(forwardReachPoint(1),forwardReachPoint(2)+wristHeight,forwardReachPoint(3));
            
            %Add the end point costs equally weighted to contribute 50% to the problem
            endPointCost1 = MocoMarkerFinalGoal('RS_endPoint',5);
            endPointCost1.setPointName('/markerset/RS');
            endPointCost1.setReferenceLocation(RS_endLoc);
            endPointCost2 = MocoMarkerFinalGoal('US_endPoint',5);
            endPointCost2.setPointName('/markerset/US');
            endPointCost2.setReferenceLocation(US_endLoc);
            endPointCost3 = MocoMarkerFinalGoal('W_endPoint',5);
            endPointCost3.setPointName('/markerset/wri_out');
            endPointCost3.setReferenceLocation(W_endLoc);

            %Add the end point cost along with an effort cost.
            mocoProblem.addGoal(endPointCost1);
            mocoProblem.addGoal(endPointCost2);
            mocoProblem.addGoal(endPointCost3);
            
        case 'HairTouch'
            
            %Get the desired end point of the movement. This will be at an
            %arbitrary point above the C7 marker (0.25m looks good for now)
            %where the middle finger needs to reach

            %Get the position of the C7 marker
            modelObject_state = modelObject.initSystem();
            C7 = modelObject.getMarkerSet().get('C7').getLocationInGround(modelObject_state);

            %Prescribe the marker end point using the X and Z coordinates of
            %the C7 marker and add the arbitrary distance to the Y position
            hairReachPoint = Vec3(C7.get(0),C7.get(1) + 0.25,C7.get(2));

            %Cleanup
            clear C7

            %Add the end point costs with appropriate weights
            endPointCost1 = MocoMarkerFinalGoal('MF_endPoint',5);
            endPointCost1.setPointName('/markerset/MiddleFinger');
            endPointCost1.setReferenceLocation(hairReachPoint);

            %Add the end point cost along with an effort cost.
            mocoProblem.addGoal(endPointCost1);
        
            
            
        %%% add other cases    
            
            
    end
    
    
end