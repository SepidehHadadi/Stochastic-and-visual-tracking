%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Distance function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [minDist, ind] = distanceFunction(inputVector, weightMatrix, nb_clusters)

%% Initialisation
%%% Allocation to compute all distances
dist = zeros(nb_clusters,1);

%% Algorithm
%%% Compute the distances
for i = 1:nb_clusters
    dist(i) = sum((weightMatrix(:,i) - inputVector') .*(weightMatrix(:,i) - inputVector'));
end

%%% return the minimum distance and the cluster associated
[minDist,ind] = min(dist);