% releaseExampleScript.m
% example script - produces a log file with time stamps of how long
% function took
%
% NOTE: type 'help functioname' to get more information on the use case for
% the functions

clear
clc
close all
% setup marker files 
load('markernames.mat');
addpath('Utilities');
extractTRCmarkerfiles('exampleData',markernames,1);
% obtain EMG results and FT sensor force data
filterEMGPlusForce('exampleData');

% run opensim IK, Kinematics, ID, and static optimization (fmincon)
starttime = datetime('now'); [h,m,s] = hms(starttime);

diary runopensimdiary.log
disp(['DIARYLOG: Start function runOpenSim for ideal force generators. Time - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
diary off

% run for ideal force generators
[~,~] = runOpenSim('exampleData','newULwMass4DoF37musc.osim','optimsampledt',3);

diary runopensimdiary.log
endtime = datetime('now'); [h,m,s] = hms(endtime-starttime);
disp(['Time taken for markerdata folder for ideal force generator in hours:minutes:seconds - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
[h,m,s] = hms(endtime);
disp(['DIARYLOG: End function runOpenSim for ideal force generators. Time - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
diary off

% run for flv force generation
addpath('Utilities');
diary runopensimdiary.log
starttime2 = datetime('now'); [h,m,s] = hms(starttime2);
disp(['DIARYLOG: Start function runOpenSim for flv curve force generators. Time - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
diary off

% run for flv curve force generators
[~,~] = runOpenSim('exampleData','newULwMass4DoF37musc.osim','optimsampledt',3,'forcegenmethod','flv-curve');

diary runopensimdiary.log
endtime2 = datetime('now'); [h,m,s] = hms(endtime2-starttime2);
disp(['Time taken for markerdata folder for flv curve force generator in hours:minutes:seconds - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
[h,m,s] = hms(endtime2);
disp(['DIARYLOG: End function runOpenSim for flv curve force generators. Time - ' num2str(h) ':' num2str(m) ':' num2str(fix(s))]);
diary off