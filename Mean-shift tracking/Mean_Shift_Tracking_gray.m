%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        MEAN SHIFT TRACKING
% ------------------------------------------------------------------------
%       Author: Oscar E. Ramos Ponce - Mohammad Ali Mirzaei
%               MSc Computer Vision and Robotics
%               Universite de Bourgogne, December 2010
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% In particular, you have to create the different functions:
%	- cd = color_distribution(imagePatch, m)
%	- k = compute_bhattacharyya_coefficient(p,q)
%	- weights = compute_weights(imPatch, qTarget, pCurrent, Nbins)
% 	- z = compute_meanshift_vector(imPatch, prev_center, weights)
%
% the function to extract an image part is given.
% ----------------
% Authors: Oscar E. Ramos Ponce - Mohammad Ali Mirzaei
% Date: 8 December 2010
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Mean_Shift_Tracking_gray

clc; clear all; close all

%Path and extension of the imagefiles
imPath = './car'; imExt = 'jpg';

%   LOAD THE IMAGES
%=========================================================================

%Check if directory and files exist
if isdir(imPath) == 0
    error('USER ERROR : The image directory does not exist');
end

filearray = dir([imPath filesep '*.' imExt]);   % get all files in the directory with the specified extension
NumImages = size(filearray,1);                  % get the number of images
if NumImages < 0
    error('No image in the directory');
end

% Load for Gray images (third dimension is the number of the image sequence)
disp('Loading image files from the video sequence, please be patient...');
for t=1:NumImages
    imgname = [imPath filesep filearray(t).name];   % get image name
    ImSeq(:,:,t) = imread(imgname);                 % load image
end
disp(' ... OK!');



%   INITIALIZE THE TRACKER
%=========================================================================

VIDEO_WIDTH  = size(ImSeq,2);    % Total number of columns
VIDEO_HEIGHT = size(ImSeq,1);    % Total number of rows



%%              BACKGROUND SUBTRACTION METHOD
%%*************************************************************************

% Initialization of the background to 0 (for avoiding array size change in time)
I_Background = zeros(VIDEO_HEIGHT,VIDEO_WIDTH);
disp('Computing the background, please be patient ...');
I_Background =  median( ImSeq, 3);
disp(' ... Background computed');
figure, imshow(I_Background), drawnow;


%%              BOUNDING BOX AROUND THE OBTAINED REGION
%   This gives the initial state for the following tracking and it consists
%   on obtaining the ROI using morphological operations
%%*************************************************************************

%Image Thresholding
Im1 = abs(double(ImSeq(:,:,1))-double(I_Background));   % Substract the first image from the background
Th = 80;                                    % Threshold value 
Im1(Im1<Th)=0; Im1(Im1>=Th)=1;              % Threshold the Image
Im1th = logical(Im1);                       % Convert the thresholded image to logical
Im1th = imclose(Im1th,strel('line',10,45)); % Close the image for some elements that appear as not connected (to get a better bounding box)
% figure, imshow(Im1th)                      % Show the thresholded image

%Get the desired ROI
[Im1Labels,NLabels] = bwlabel(Im1th,8);         % Label the thresholded image
for n=1:NLabels
   sizeLabel(n) = length(find(Im1Labels==n));   % Count the elements in each label
end
[~,MaxLabel] = max(sizeLabel);                  % Find the label with more elements (ROI)

[Xreg,Yreg] = find(Im1Labels==MaxLabel);        % Find the position of pixels belonging to the ROI
Xmax = max(Xreg); Xmin = min(Xreg);             % x extremes of the bounding box 
Ymax = max(Yreg); Ymin = min(Yreg);             % y extremes of the bounding box 
ROI_Center = [ (Xmin+Xmax)/2, (Ymin+Ymax)/2];   % Center of the ROI (mean of the extremes) 
ROI_Height = Xmax - Xmin;                       % Height of the ROI
ROI_Width  = Ymax - Ymin;                       % Width of the ROI

%Show the ROI on the image
figure, imshow(ImSeq(:,:,1)); hold on;
line([Ymin Ymin],[Xmin Xmax]), line([Ymax Ymax],[Xmin Xmax])
line([Ymin Ymax],[Xmin Xmin]), line([Ymin Ymax],[Xmax Xmax])
hold off

% %Show the thresholded images
% figure
% subplot(1,2,1), imshow(Im1th)
% subplot(1,2,2), imshow(Im1Labels==MaxLabel)


%%*************************************************************************
%%                          MEANSHIFT TRACKING
%*************************************************************************

%DEFINITION OF THE COLOR MODEL OF THE OBJECT: TARGET MODEL

%Extract the image patch of the ROI for the target model
imPatch = extract_image_patch_center_size(ImSeq(:,:,1), ROI_Center, ROI_Width, ROI_Height);
%figure, imshow(imPatch)

%Compute the color distribution in RGB color space
Nbins = 8;
TargetModel = color_distribution(imPatch, Nbins);

% Mean-Shift Algorithm 
prev_center = ROI_Center;       % set the location to the previous one 
figure;
for n = 2:NumImages

    I = ImSeq(:,:,n);           % get next frame
    %while(1)
    for i=1:5
        % STEP 1: calculate the pdf of the previous position
    	imPatch    = extract_image_patch_center_size(I, prev_center, ROI_Width, ROI_Height);
    	ColorModel = color_distribution(imPatch, Nbins);
    
    	% evaluate the Bhattacharyya coefficient
     	rho_0 = compute_bhattacharyya_coefficient(TargetModel, ColorModel);
    
    	% STEP 2: derive the weights
    	weights = compute_weights(imPatch, TargetModel, ColorModel, Nbins);
    
    	% STEP 3: compute the mean-shift vector using Epanechnikov kernel (weighted average)
        z = compute_meanshift_vector(imPatch, prev_center, weights);
      	new_center = z;
            
    	% STEP 4: Compute coefficient for the new center
        imPatchNew    = extract_image_patch_center_size(I, new_center, ROI_Width, ROI_Height);
    	ColorModelNew = color_distribution(imPatchNew, Nbins);
        rho_1 = compute_bhattacharyya_coefficient(TargetModel, ColorModelNew);
        
        % STEP 5: If new coefficient is smaller than previous one, update center
        if (rho_1 < rho_0)
            new_center = 0.5*(new_center + prev_center);
        end
        
        % STEP 6
    	if norm(new_center-prev_center, 1) < eps
       		break
    	end
    	prev_center = new_center;

    end
	
    % Show the tracking results     	           
    Xmin = round(new_center(1) - ROI_Height/2);
    Xmax = round(new_center(1) + ROI_Height/2);
    Ymin = round(new_center(2) - ROI_Width/2);
    Ymax = round(new_center(2) + ROI_Width/2);
    imshow(ImSeq(:,:,n)); hold on;
    line([Ymin Ymin],[Xmin Xmax]), line([Ymax Ymax],[Xmin Xmax])
    line([Ymin Ymax],[Xmin Xmin]), line([Ymin Ymax],[Xmax Xmax])
    hold off
    drawnow
    
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                                 THE DIFFERENT FUNCTIONS TO BE USED

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Function to extract the image patch 
function imPatch = extract_image_patch_center_size(I, c, w, h)
%This function extracts an image patch in image I given the center and size of the ROI

c = round(c);                   % Round the center values                

VIDEO_WIDTH  = size(I,2);       % Width of the original image
VIDEO_HEIGHT = size(I,1);       % Height of the original image

x = c(1)-h/2;                   % x for Top left corner of the patch
y = c(2)-w/2;                   % y for Top left corner of the patch
r = round(max(x, 1));           % If the top left corner is out of the image,
c = round(max(y, 1));           %    take (1,1) as top left

r2 = round(min(VIDEO_HEIGHT, x+h+1));   % x for Bottom right corner of the patch
c2 = round(min(VIDEO_WIDTH,  y+w+1));   % y for Bottom right corner of the patch

imPatch = I(r:r2, c:c2, :);     % Return the patch


%% Create the color distribution 
function cd = color_distribution(imagePatch, m)
%This function creates a m bins histogram to be used as distribution
%   Assumption: imagePatch has values from 0 to 255

[height,width] = size(imagePatch);  % Size of the patch
xc = ceil(height/2);                % x Center of the patch
yc = ceil(width/2);                 % y Center of the patch

sizebin = 256/m;                            % Elements in a bin, for m bins 
b = floor(double(imagePatch)/sizebin);      % Quantized image levels into m bins

Cd = pi; d = 2;                     % Constants for the model
invCd = inv(Cd);                    % Constants for the model
q = zeros(1,m);                     % Initialization of q to zeros
for x=1:height
    for y=1:width
        MagX = ((x-xc)/(height/2))^2 + ((y-yc)/(width/2))^2;    % Compute the magnitude
        if MagX<1                                               % If it is <1, update the value
            ke = 0.5*invCd*(d+2)*(1-MagX);
        else                                                    % Otherwise, assign 0 to the value
            ke = 0;
        end
        q(b(x,y)+1) = q(b(x,y)+1) + ke;                         % Sum the values
    end
end

cd = q./sum(q);                                                 % Normalization


%% Compute the Brattacharyya coefficient 
function rho = compute_bhattacharyya_coefficient(p,q)
% This function computes the Bhattacharyya distance between two 1D histograms. 
m = length(p);
rho = 0;
for u = 1:m
    rho = rho + sqrt(p(u)*q(u));
end


%% Compute the weights w_i for every pixel 
function weights = compute_weights(imPatch, qTarget, pCurrent, Nbins)
% This function computes the weights w_i, for every pixel in the target region

sizebin = 256/Nbins;                    % Size of each bin (elements in the bin)
b = floor(double(imPatch)/sizebin);     % Image divided in m bins

m = length(qTarget);                    % Number of elements in the "histogram" distribution
w = zeros(size(imPatch));               % Initiallize the weight to 0
for x=1:size(imPatch,1)
    for y=1:size(imPatch,2)
        w(x,y) = sqrt(qTarget(b(x,y)+1)/pCurrent(b(x,y)+1));
    end
end
weights = w;

%-------------------------------------

%% Compute mean shift vector
function y = compute_meanshift_vector(imagePatch, ROI_Center, weights)
% This function computes the mean shift vector y given the weights for every pixel in the ROI

[height,width] = size(imagePatch);      % Size of the patch
Xc = [height/2, width/2];               % y Center of the patch

num = 0; den = 0;                       % initialize numerator and denominator to 0
for x=1:size(imagePatch,1)              
    for y=1:size(imagePatch,2)
        MagX = ((x-Xc(1))/(height/2))^2 + ((y-Xc(2))/(width/2))^2;  % Compute the magnitude
        if MagX<1                                                   % If the magnitude is >1, sum the contributions of the weights
            num = num + weights(x,y)*[x y];
            den = den + weights(x,y);
        end
    end
end
z = num/den;                            % The new distance with respect to the patch
y = ROI_Center + z - Xc;                % The new distance with respect to the global image

