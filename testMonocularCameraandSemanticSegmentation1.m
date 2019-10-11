%https://ww2.mathworks.cn/help/driving/examples/create-occupancy-grid-using-monocular-camera-sensor.html
clc;
clear all;
close all;

classes = [
    "Sky"
    "Building"
    "Pole"
    "Road"
    "Pavement"
    "Tree"
    "SignSymbol"
    "Fence"
    "Car"
    "Pedestrian"
    "Bicyclist"
    ];


data = load('segnetVGG16CamVid.mat');
net = data.net;

T=dir('.\testImages\*.jpg');
for i = 1:length(T)
    str=[T(i).folder '\' T(i).name];
I = imread(str);
% I = imresize(I,0.5);
% Segment the image.
[C,scores,allScores] = semanticseg(I,net);

% Overlay free space onto the image.
B = labeloverlay(I,C);

% Display free space and image.
figure
imshow(B)


roadClassIdx = 4;
freeSpaceConfidence = allScores(:,:,roadClassIdx);
    
% Display the free space confidence.
figure
imagesc(freeSpaceConfidence)
title('Free Space Confidence Scores')
colorbar
pause
end