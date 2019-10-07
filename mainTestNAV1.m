function mainTestNAV1
%matlabµ¼º½Ä£¿éµÄgetting start
clear all;
close all;
addpath('D:\Documents\MATLAB\Examples\R2019b\shared_positioning\QuaternionExample\')
dr = HelperDrawRotation;
dr.drawTeapotRotations;

figure;
dr.draw3DOrientation(gca, [1/3 2/3 2/3], 30);
end