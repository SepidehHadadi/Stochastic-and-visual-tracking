function [xt,Vt]=modifiedMShift(im0,xt,V,histO,K,beta,varargin)

[data,rX,rY]=GetAffineRegion(im0,xt,V,K.rX,K.rY,K.sigmas);


[histR]=ColorHist(data,histO.nBinsD,K.rK);
[riNonZero]=find(histR.data>0);%prevent divide by zero
rWeigths=(histR.H(:,riNonZero)*sqrt(histO.data(riNonZero)./histR.data(riNonZero))')'.*K.rK;
sumW=sum(rWeigths);
if (sumW>0)
    rQs=rWeigths/sumW;
else
    rQs=rWeigths; 
end
        
dx=[sum(rQs.*rX);sum(rQs.*rY)];
xt=xt+dx;
Vxy=sum(rQs.*rX.*rY);
Vt=beta*[sum(rQs.*rX.*rX),Vxy;Vxy,sum(rQs.*rY.*rY)];


%clip
Vt(1,1)=max(Vt(1,1),1);Vt(2,2)=max(Vt(2,2),1);%clip to min 1
