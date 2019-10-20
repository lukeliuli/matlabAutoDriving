% https://www.mathworks.com/help/vision/examples/semantic-segmentation-using-deep-learning.html
clc;
clear all;
close all;

pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/deeplabv3plusResnet18CamVid.mat';
pretrainedFolder = '.\';
pretrainedNetwork = fullfile(pretrainedFolder,'deeplabv3plusResnet18CamVid.mat'); 
if ~exist(pretrainedNetwork,'file')
    mkdir(pretrainedFolder);
    disp('Downloading pretrained network (58 MB)...');
    websave(pretrainedNetwork,pretrainedURL);
end

% pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/deeplabv3plusResnet50CamVid.mat';
% pretrainedFolder = '.\';
% pretrainedNetwork = fullfile(pretrainedFolder,'deeplabv3plusResnet101CamVid.mat'); 
% if ~exist(pretrainedNetwork,'file')
%     mkdir(pretrainedFolder);
%     disp('Downloading pretrained network (58 MB)...');
%     websave(pretrainedNetwork,pretrainedURL);
% end

data = load('deeplabv3plusResnet18CamVid.mat');
net = data.net;
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

T=dir('.\testImages\*.jpg');
for i = 1:length(T)
    str=[T(i).folder '\' T(i).name];
    I = imread(str);
    I = histeq(I);
 I = imresize(I,[720 960]);
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