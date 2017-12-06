%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Weight update function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [newWeightMatrix] = weightUpdateFunction(weightMatrix, cluster, learningRate, vector, size_vector)

%% Initialisation
newWeightMatrix = weightMatrix;

%% Updating
newWeightMatrix(:,cluster) = newWeightMatrix(:,cluster) + (learningRate*(vector' - newWeightMatrix(:,cluster)));