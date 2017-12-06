function R=FitGauss(im0,s,histO,K)


nStates=length(s);
R=eye(nStates);
fClip=0.005;

nSamples=2;
rSamples=zeros(1,nSamples);

[xt,V]=SToMeanVar(s);
rho0=Similarity(im0,xt,V,histO,K);

rD=[2 0 0 0 0;-2 0 0 0 0]';
[xt,V]=SToMeanVar(s+rD(:,1));
rSamples(1)=rho0-Similarity(im0,xt,V,histO,K);
[xt,V]=SToMeanVar(s+rD(:,2));
rSamples(2)=rho0-Similarity(im0,xt,V,histO,K);
rSamples(find(rSamples<fClip))=fClip;%clip negative
R(1,1)=mean(sum(rD.^2,1)./rSamples);

rD=[0 2 0 0 0;0 -2 0 0 0]';
[xt,V]=SToMeanVar(s+rD(:,1));
rSamples(1)=rho0-Similarity(im0,xt,V,histO,K);
[xt,V]=SToMeanVar(s+rD(:,2));
rSamples(2)=rho0-Similarity(im0,xt,V,histO,K);
rSamples(find(rSamples<fClip))=fClip;%clip negative
R(2,2)=mean(sum(rD.^2,1)./rSamples);

%var
rD=0.1*[0 0 s(3) 0 0;0 0 -s(3) 0 0]';
[xt,V]=SToMeanVar(s+rD(:,1));
rSamples(1)=rho0-Similarity(im0,xt,V,histO,K);
[xt,V]=SToMeanVar(s+rD(:,2));
rSamples(2)=rho0-Similarity(im0,xt,V,histO,K);
rSamples(find(rSamples<fClip))=fClip;%clip negative
R(3,3)=mean(sum(rD.^2,1)./rSamples);

rD=0.1*[0 0 0 s(4) 0;0 0 0 -s(4) 0]';
[xt,V]=SToMeanVar(s+rD(:,1));
rSamples(1)=rho0-Similarity(im0,xt,V,histO,K);
[xt,V]=SToMeanVar(s+rD(:,2));
rSamples(2)=rho0-Similarity(im0,xt,V,histO,K);
rSamples(find(rSamples<fClip))=fClip;%clip negative
R(4,4)=mean(sum(rD.^2,1)./rSamples);

rD=0.1*[0 0 0 0 s(5);0 0 0 0 -s(5)]';
[xt,V]=SToMeanVar(s+rD(:,1));
rSamples(1)=rho0-Similarity(im0,xt,V,histO,K);
[xt,V]=SToMeanVar(s+rD(:,2));
rSamples(2)=rho0-Similarity(im0,xt,V,histO,K);
rSamples(find(rSamples<fClip))=fClip;%clip negative
R(5,5)=mean(sum(rD.^2,1)./rSamples);

pi=rho0;

