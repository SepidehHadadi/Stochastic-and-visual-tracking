close all
clear all
clc

%% load data
 load('synthetic_model.mat');
 
 I1 = im2double(rgb2gray(imread('yellowstone_left.png')));
I2 = im2double(rgb2gray(imread('yellowstone_right.png')));

cvexShowImagePair(I1, I2, 'Left image', 'Right image');
cvexShowMatches(I1, I2, [], [], 'Left image', 'Right image');
%% complete the processes
% feature points detection
hcornerdet = vision.CornerDetector('MaximumCornerCount', 500, 'CornerThreshold', 0.001, 'NeighborhoodSize', [21 21]);
cornerPoints1 = step(hcornerdet, I1);
cornerPoints2 = step(hcornerdet, I2);
cvexShowImagePair(I1, I2, 'Corners in left image', 'Corners in right image', 'SingleColor', cornerPoints1, cornerPoints2);

% feature selection
[features1, validPoints1] = extractFeatures(I1, cornerPoints1, 'BlockSize', 9);
[features2, validPoints2] = extractFeatures(I2, cornerPoints2, 'BlockSize', 9);
[indexPairs, match_metric] = matchFeatures(features1, features2, 'Metric', 'normxcorr', 'MatchThreshold', 5);

% Retrieve locations of matched points for each image
matchedPoints1 = validPoints1(indexPairs(:, 1), :);
matchedPoints2 = validPoints2(indexPairs(:, 2), :);

cvexShowMatches(I1, I2, matchedPoints1, matchedPoints2, 'Matched points in I1', 'Matched points in I2');
[fMatrix, inliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC',  'NumTrials', 1000, 'DistanceThreshold', 0.1, 'Confidence', 99.99);

inlierPoints1 = matchedPoints1(inliers, :);
inlierPoints2 = matchedPoints2(inliers, :);

cvexShowMatches(I1, I2, inlierPoints1, inlierPoints2, 'Inlier points in I1', 'Inlier points in I2');

if isEpipoleInImage(fMatrix, size(I1)) ...
    || isEpipoleInImage(fMatrix', size(I2))
    error(['For the rectification to succeed, the epipoles must be ',...
        'outside the images.']);
end

[t1, t2] = estimateUncalibratedRectification(fMatrix, ...
    inlierPoints1, inlierPoints2, size(I2));

cvexShowMatches(I1, I2, inlierPoints1, inlierPoints2, ...
    'Inlier points in I1', 'Inlier points in I2', 'RC', t1, t2);
cvexShowStereoImages('Rectified image 1', 'Rectified image 2',...
    I1, I2, inlierPoints1, inlierPoints2, fMatrix, t1, t2);

 det_F_normalized_8point(X1,X2)
