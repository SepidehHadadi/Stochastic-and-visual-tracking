%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Training Kohonen Network
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [classification] = testKohonenNetwork(data, weightMatrix)

%% Initalisation

%%% Find the number of clusters
nb_clusters = size(weightMatrix,2);

%%% Find the number of vectors
nb_vectors = size(data,1);

%% Begin the algortihm
%%% Allocation of the data
classification = zeros(nb_vectors,2);
%% Start the testing
%%% For each vectors
for i = 1:nb_vectors
    %%% Compute the distance
    [dist,clus] = distanceFunction(data(i,:),weightMatrix,nb_clusters);
    
    %%% Assign to the good clusters
    classification(i,1) = clus;
    %%% Put the distance in option
    classification(i,2) = dist;
end
%% End of the training
