%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Advanced Robot Motion Planning Project - Fall 2018
% RRT* information maximization route planning
% Collin, Jorge, George
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clean up
clc; clear all; close all;

%% Initialize parameters
gridsize = 100;     % dimension of a square gridmap
range = [-1, 0, 1]; % range of possible values for the map
threshold = 0;      % threshold for passable grid cells

%% Initialize the grid