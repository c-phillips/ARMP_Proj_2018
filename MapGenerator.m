%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Advanced Robot Motion Planning Project - Fall 2018
% RRT* information maximization route planning
% Collin, Jorge, George
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clean up
clc; clear all; close all;

%% Initialize parameters
worldsize = 100;
gridsize = 25;          % dimension of a square gridmap
threshold = 0.2;        % threshold for passable grid cells

samplesize = worldsize/gridsize;

%% Create the map

% First, lets generate some random points for a voroniDiagram
gen = [(rand(1000,1)*worldsize)-worldsize/2,...
       (rand(1000,1)*worldsize)-worldsize/2];

% We create the voronoiDiagram from a delaunayTriangulation because
% eventually we might make the region labeling smarter by considering
% adjacent regions
dt = delaunayTriangulation(gen(:,1),gen(:,2));
[v, r] = voronoiDiagram(dt);

% initialize the labels and polygons, then we extract the polygons from the
% generated diagrams and randomly sample labels
labels = zeros(1,length(r));
polygons = {};
for i = 1:length(r)
    for j = 1:length(r{i})
        polygons{i} = v(r{i},:);
    end
    % I have selected the lables somewhat arbitrarily,
    % 0 - no information
    % 5 - some information
    % 10- high information
    labels(i) = randsample([0, 5, 10],1);
end

% Create a meshgrid for generating our map from the polygons
sample_x = [0:1/samplesize:worldsize];
sample_y = sample_x;
[X, Y] = meshgrid(sample_x, sample_y);

% Here we check every grid point and assign its value to equal the label of
% the polygon it is colliding with
collision_map = zeros(length(X),length(Y));
for i = 1:length(polygons)
    in = inpolygon(X,Y, polygons{i}(:,1), polygons{i}(:,2));
    collision_map = collision_map + labels(i)*in;
end

% figure()
% image(collision_map)
% xlim([0 worldsize])
% ylim([0 worldsize])

% Now we reduce the information by mean-pooling points in the collision map
map = zeros(gridsize,gridsize);
for i = 1:gridsize
    for j = 1:gridsize
        box_x = i*samplesize;
        box_y = j*samplesize;
        map(i,j) = mean2(collision_map(...
            box_x-samplesize+1:box_x, box_y-samplesize+1:box_y));
    end
end

% Replace all the values in the map below the threshold with 0
normMap = map - min(map(:));
normMap = normMap ./ max(normMap(:));
normMap(normMap < threshold) = 0;

figure(3)
image(normMap.*50)  % its scaled just to see the colors





