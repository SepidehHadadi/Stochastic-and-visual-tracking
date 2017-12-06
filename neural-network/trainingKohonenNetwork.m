%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Training Kohonen Network
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [weightMatrix] = trainingKohonenNetwork(data, nb_clusters, nb_iter, radiusLearning, initialLearningRate)

%% Initalisation

%%% Find the size of vectors
size_vector = size(data,2);

%%% Find the number of vectors
nb_vectors = size(data,1);

%%% Initial weight matrix
%%% Random weight matrix fill following Gaussian distribution
weightMatrix = random('Normal',0,1,size_vector,nb_clusters);
weightMatrix = (weightMatrix - min(min(weightMatrix)));
weightMatrix = weightMatrix/(max(max(weightMatrix)));
%%% This matrix should be fill randomly
% weightMatrix = zeros(size_vector,nb_clusters);
%weightMatrix = [ 0.2 , 0.8 ; 0.6 , 0.4 ; 0.5 , 0.7 ; 0.9 , 0.3 ];

%% Begin the algortihm
%% Start the training
learningRate = initialLearningRate;
for j = 1:nb_iter
    %%% For each vectors
    for i = 1:nb_vectors
        %%% Compute the distance
        [dist,clus] = distanceFunction(data(i,:),weightMatrix,nb_clusters);

        %%% Update the weight matrix
        [weightMatrix] = weightUpdateFunction(weightMatrix, clus, learningRate, data(i,:), size_vector);
    end
    %%% Update the learning rate
    learningRate = learningRateFunction(j-1,initialLearningRate);
end
%% End of the training
