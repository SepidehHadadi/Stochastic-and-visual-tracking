%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kohonen neural network
%%% Guillaume Lemaitre
%%% -----------------------------------------------------------------------
%%% Real data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialisation
clear all;
close all;
clc;

addpath('data');

%% Training
%%% Parameters
nb_clusters = 2;
initialLearningRate = 0.6;
nb_iterations = 100;
radiusLearning = 0;

%%% Open the data
%trainingData = [1 1 0 0 ; 0 0 0 1 ; 1 0 0 0 ; 0 0 1 1];
trainingData = load('patient.txt');
trainingData = [trainingData ; load('control.txt')];

%%% Normalization between 0 and 1
trainingData = mat2gray(trainingData);
tic
%%% Train a Kohonen network
[weightMatrix] = trainingKohonenNetwork(trainingData, nb_clusters, nb_iterations, radiusLearning, initialLearningRate);

toc

%% Testing

%%% Open the data
%testData = [0 0 0 0.9 ; 0 0 0.8 0.9 ; 0.7 0 0 0 ; 0.7 0.9 0 0];
testData = trainingData;load('control.txt');

[classification] = testKohonenNetwork(testData, weightMatrix);

