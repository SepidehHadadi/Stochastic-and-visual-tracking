%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Learning rate function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [newVal] = learningRateFunction(iter, initialValue)

newVal =  initialValue*(0.96)^(iter);