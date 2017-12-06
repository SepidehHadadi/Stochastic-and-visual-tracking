%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MEAN SHIFT TRACKING: a simple color-histogram tracking demo script
% ------------------------------------------------------------------------
%       Author: Mohammad Ali Mirzaei - Oscar E. Ramos Ponce
%               MSc Computer Vision and Robotics
%               Universite de Bourgogne, December 2010
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Date: 8/12/2010

clear
close all;
clear all;

%% LOAD THE IMAGES
%=========================================================================

type   = 'bmp';
fplace = './head';     %'./car_ng';
files  = dir(fplace);
filename0 = strcat(fplace,filesep,files(4).name);

[m,n,k1] = size(imread(filename0,type));% for plane project

background = zeros(m,n); 
fz = size(files,1);
Images = uint8(zeros(m,n,k1,fz));
for k=4:fz-1
    filename=strcat(fplace,filesep,files(k).name);
    IN =imread(filename);
    Images(:,:,:,k-3)=im2uint8(IN);
end
if(k1==1)
    [X, map] = gray2ind(IN, 256);
else
    if(k1==3)
    [X, map] = rgb2ind(IN, 256);
    end
end
% Mv=immovie(Images,map);

%choose the tracker
dScale=0.1;
sTracker='colormeanshifttracker'
beta=1.2;
nIterationMax=6;


%% Initialization and ROI selection
%select object and define the place of mask


%background calculation
%data=Mv;
im0=Images(:,:,:,1);
I1IN=[];
for i=1:m
    for j=1:n
        background(i,j)=median(Images(i,j,3,:));
    end
end
BKG=double(background);


%Image Thresholding
ImSeq1=rgb2gray(Images(:,:,:,1));
Im1 = abs(double(ImSeq1)-BKG); % Substract the first image from the background
if(k1==1)
    Th = 80;                                    % Threshold value 
else
    if(k1==3)
        Th = 50; 
    end
end
Im1(Im1<Th)=0; Im1(Im1>=Th)=1;              % Threshold the Image
Im1th = logical(Im1);                       % Convert the thresholded image to logical

Im1th = imclose(Im1th,strel('line',10,45)); % Close the image for some elements that appear as not connected (to get a better bounding box)
    
%Get the desired ROI
[Im1Labels,NLabels] = bwlabel(Im1th,8);         % Label the thresholded image
for n1=1:NLabels
   sizeLabel(n1) = length(find(Im1Labels==n1));   % Count the elements in each label
end
[MaxVal,MaxLabel] = max(sizeLabel);             % Find the label with more elements (ROI)

[Xreg,Yreg] = find(Im1Labels==MaxLabel);        % Find the position of pixels belonging to the ROI
Xmax = max(Xreg); Xmin = min(Xreg);             % x extremes of the bounding box 
Ymax = max(Yreg); Ymin = min(Yreg);             % y extremes of the bounding box 
ROI_Center = [ (Xmin+Xmax)/2, (Ymin+Ymax)/2];   % Center of the ROI (mean of the extremes) 
ROI_Height = Xmax - Xmin;                       % Height of the ROI
ROI_Width  = Ymax - Ymin; 

imMask=zeros(m,n);
imMask(Xmin:Xmax,Ymin:Ymax)=1;[ry,rx]=find(imMask);
m0=mean([rx,ry])';V0=cov([rx,ry]);


%----------------------------end of mask definition-----------------------------
%% make object histogram 
[K]=GaussKernel(V0);%make kernel of appropriate size
data=GetAffineRegion(im0,m0,V0,K.rX,K.rY,K.sigmas);
nBinsD=4;histO=ColorHist(data,nBinsD,K.rK);

%state vector
s0=MeanVarToS(m0,V0);nStates=length(s0);s=s0;
%% setting for modification


%dynamic model s=A*s+W
A=eye(nStates);
W=[3^2  0   0   0   0;
    0   3^2 0   0   0;
    0   0   1^2 0   0;
    0   0   0   1^2 0;
    0   0   0   0   1^2];
lambda=0.2;%observation model p~exp(-0.5*(1-rho)/lambda^2)
Pk=100^2*eye(nStates);%some high initial value for the variance

%% tracking algorithm 

for iFrame=1:size(files,1)-2
%     data=aviread(sAvi,iFrame);
im0=Images(:,:,:,iFrame);%read an image
    %different trackers
    switch sTracker
        case 'colormeanshifttracker'
            s=A*s;
            Pk=A*Pk*A'+W;
            %find max
            [xt,V]=SToMeanVar(s);%prediction as starting point
            for iIteration=1:nIterationMax
                [xt,V]=modifiedMShift(im0,xt,V,histO,K,beta);
            end
            z=MeanVarToS(xt,V);
            R=FitGauss(im0,z,histO,K)*lambda^2;
            %Kalman filter
            KFgain=Pk*inv(Pk+R);
            s=s+KFgain*(z-s);
            Pk=(eye(nStates)-KFgain)*Pk;
        case 'MeanshiftTracker'
            %predict
            s=A*s;
            Pk=A*Pk*A'+W;
            %Find the local max using the prediction as the start point
            [xt,V]=SToMeanVar(s);%prediction as starting point
            for iIteration=1:nIterationMax
                xt=modifiedMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho0=Similarity(im0,xt,V,histO,K);
            z0=MeanVarToS(xt,V);
            %measure V by trying a number of different scales
            rD=dScale*[0 0 0 0 0;0 0 s(3) 0 s(5); 0 0 -s(3) 0 -s(5)]';
            %try bigger
            [xt,V]=SToMeanVar(s+rD(:,2));
            for iIteration=1:nIterationMax
                xt=modifiedMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho1=Similarity(im0,xt,V,histO,K);
            z1=MeanVarToS(xt,V);
            %try smaller
            [xt,V]=SToMeanVar(s+rD(:,3));
            for iIteration=1:nIterationMax
                xt=modifiedMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho2=Similarity(im0,xt,V,histO,K);
            z2=MeanVarToS(xt,V);
            %choose the best
            [dummy,ri]=max([rho0 rho1 rho2]);
            if (ri(1)==1) z=z0; end;
            if (ri(1)==2) z=z1; end;
            if (ri(1)==3) z=z2; end;
            
            R=FitGauss(im0,z,histO,K)*lambda^2;
            %since we can be far from a local minima for the V
            %the local curvature is not good estimate for the V uncertainty !!! 
            %-use some fixed value - mean value from the EMS for example
            R(3,3)=1;R(4,4)=1;R(5,5)=1;
            %Kalman filter equations
            KFgain=Pk*inv(Pk+R);
            s=s+KFgain*(z-s);
            Pk=(eye(nStates)-KFgain)*Pk;    
    end

    %show
    figure(1),imshow(im0);hold on;%imshow is from the Image processing toolbox
    h=PlotEllipse(s);set(h,'LineWidth',4,'Color',[1 0.7 1],'LineStyle','--');
    drawnow;hold off
end