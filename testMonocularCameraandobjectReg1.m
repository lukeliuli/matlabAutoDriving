doTraining = false;
if ~doTraining && ~exist('yolov2ResNet50VehicleExample.mat','file')
    % Download pretrained detector.
    disp('Downloading pretrained detector (98 MB)...');
    pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/yolov2ResNet50VehicleExample.mat';
    websave('yolov2ResNet50VehicleExample.mat',pretrainedURL);
end

load('yolov2ResNet50VehicleExample.mat')

T=dir('.\testImages\*.jpg');
for i = 1:length(T)
    str=[T(i).folder '\' T(i).name];
I = imread(str);
 I = imresize(I,[360 480]);
% Segment the image.
% Run the detector.
[bboxes, scores] = detect(detector, I);


% Annotate detections in the image.
I = insertObjectAnnotation(I, 'rectangle', bboxes, scores);
figure
imshow(I)
end