%% This script runs all processes encapsulated in this project.
%
%  Firstly, the models of altered strength (i.e. 20% stronger and weaker)
%  of the different muscle groups are created.
%
%  Secondly, the predictive simulations for the different tasks are run.
%  These can be commented in/out for the specific tasks of interest.
%
%  Thirdly, the simulation results for the different tasks are compiled.
%
%  NOTE: This script should be run from the 'Code' directory of the
%  project.

clear; clc;

%Allocate current path to variable
defaultPath = cd;

%Add the main and supplementary code to path
addpath([defaultPath,'\Main']);
addpath([defaultPath,'\Supplementary']);

%Set the directory that contains the models
cd('..\Models');
modelPath = cd;

%% Create the altered strength models

%Set the scale factors
%Currently 80% and 120% of baseline strength
scaleFactors = [0.8,1.2];

%Set the baseline model
baselineModel = [modelPath,'\BaselineModel.osim'];

%Run the create models pipeline
ShoulderStrengthSims_1_GenerateModels(baselineModel,scaleFactors);

%% Run the simulations for the specified tasks
%
%  Tasks with a concentric portion need to be run before eccentric
%  portions due to concentric end states being used as initial states for
%  eccentric portion of the movement
%
%  Note that running all of these predictive simulations at once will take
%  a substantial amount of computing time.

clc;

%Load in the task bounds data for simulations
cd('..\SupplementaryInfo');
taskBounds = load('Vidt_MovementValues.mat');

%Set results directory for simulations
cd('..\SimulationResults');
resultsDir = cd;

%%

%Run the tasks
% % % ShoulderStrengthSims_2_RunSimulations('ConcentricUpwardReach90',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('ConcentricForwardReach',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('ConcentricHairTouch',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('EccentricUpwardReach90',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('EccentricForwardReach',modelPath,resultsDir,taskBounds,'weakened');
ShoulderStrengthSims_2_RunSimulations('ConcentricUpwardReach105',modelPath,resultsDir,taskBounds,'weakened');
ShoulderStrengthSims_2_RunSimulations('EccentricUpwardReach105',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('ConcentricRearTouch',modelPath,resultsDir,taskBounds,'weakened');
% % % ShoulderStrengthSims_2_RunSimulations('ConcentricAxillaTouch',modelPath,resultsDir,taskBounds,'weakened');

%%


%Collate the results
ShoulderStrengthSims_3_CalculateVariables('ConcentricUpwardReach90',50,'weakened',resultsDir,modelPath);
ShoulderStrengthSims_3_CalculateVariables('ConcentricForwardReach',50,'weakened',resultsDir,modelPath);
ShoulderStrengthSims_3_CalculateVariables('ConcentricHairTouch',50,'weakened',resultsDir,modelPath);



%%


cd('..\CompiledSimulations');
compiledDir = cd;

%Visualise results
ShoulderStrengthSims_4_VisualiseResults('ConcentricUpwardReach90','weakened');
ShoulderStrengthSims_4_VisualiseResults('ConcentricForwardReach','weakened');
ShoulderStrengthSims_4_VisualiseResults('ConcentricHairTouch','weakened')






