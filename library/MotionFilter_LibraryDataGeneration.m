% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% DESCRIPTION
% This script generates input and output busses for the MotionFilter_Library and stores all generated signals and simulink bus
% objects to the MotionFilter_LibraryData simulink data dictionary.
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Ensure that the workspace is cleared to force a complete re-creation of any data
clear all;


% structure formats
structMotionFilter3DoFIn = motionfilter.MotionFilter3DoF.GetInputStructFormat();
structMotionFilter3DoFOut = motionfilter.MotionFilter3DoF.GetOutputStructFormat();


% create all simulink bus objects
[~] = ave.Struct2Bus(structMotionFilter3DoFIn,'busMotionFilter3DoFIn');
[~] = ave.Struct2Bus(structMotionFilter3DoFOut,'busMotionFilter3DoFOut');


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% CREATE DATA DICTIONARY
% Use all variables in the base workspace and put them into a data dictionary.
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
varNames = who();
thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
filenameDataDictionary = fullfile(thisDirectory, 'MotionFilter_LibraryData.sldd');
ave.CreateDataDictionaryFromBaseWorkspaceVariables(filenameDataDictionary, varNames);

% refresh library
% filenameLibrary = fullfile(thisDirectory, 'MotionFilter_Library.slx');
% Simulink.LibraryDictionary.refresh(filenameLibrary);

