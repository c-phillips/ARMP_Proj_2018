%% Cleanup before running
clear all; clc; close all;

%% Configure program parameters
map = imread('good_map.png');       % read in the map image file
score_map = im2double(map);         % we need to convert the image to a numeric value
threshold = min(map(:))+15;         % this is the threshold used for obstacle avoidance

color_map = cat(3, map, map, map);  % this map is just for display purposes

[map_x, map_y] = size(map);         % grabbing the size of the map

% Here is where we set the main parameters:
% alpha(1) = weighting for nearest node
% alpha(2) = weighting for the normalized path information
% alpha(3) = weighting for the distance from the new point to the goal

% alpha = [0.1, 200, 0.05];
alpha = [1, 0, 0];
epsilon = 1;                        % the amount to step by to the next point
capture_radius = epsilon*5;

% LIMITS
x_lim = [1, map_x];
y_lim = [1, map_y];
sample_spacing = 1;

% I pretty arbitrarily picked these:
startpos = [5,5];
goalpos = [310,270];

%% Create the obstacles
x_grid = x_lim(1)+epsilon:sample_spacing:x_lim(2)-epsilon;
y_grid = y_lim(1)+epsilon:sample_spacing:y_lim(2)-epsilon;

disp('marking obstacles')
for i = 1:map_x
    for j = 1:map_y
        % if the information value is below the threshold mark it red
        if(map(i,j) <= threshold)
            color_map(i,j,1) = 134;
            color_map(i,j,2) = 31;
            color_map(i,j,3) = 65;
        end
    end
end

%% Generate the tree
% The nodes array stores all of the node positions, as well as their parents:
% node = [parent, position[x,y]]
nodes = [];
nodes(1,:) = [0, startpos];

% The path_info array stores all of the path information sums up to a node and its length:
% path_info(i) = [length_to_i, info_to_i]
path_info = [];
path_info(1,:) = [1, score_map(startpos(2),startpos(1))];

found_path = 0;
it = 0;
fprintf('finding path... \n\n\r')
disp('\n\r')
while found_path == 0
    % This section is just to show progress
    it = it + 1;
    for i = 1:numel(num2str(it))
        fprintf('\b')
    end
    fprintf('\b\b\b\bit: %d',it)
    
    % We sample a new random point on the map
    v_samp = [0,0];
    v_samp(1) = datasample(x_grid,1);
    v_samp(2) = datasample(y_grid,1);
    
    % pos is all of the node positions for the entire tree
    pos = nodes(:,2:3);
    
    % in order to do the operations for all matrix entries at once, we need some large matrices of repeated values
    rep_samp = repmat(v_samp,size(pos(:,1:2),1),1);
    rep_goal = repmat(goalpos,size(pos(:,1:2),1),1);
    
    diff = rep_samp-pos(:,1:2); % we use this a few times in qualities
    
    % This line is super long, but it performs all of the node calculations at once which is very convenient.
    % The terms are clearly multiplied by their alpha values
    % term1 = distance from the sampled node to all nodes in the tree
    % term2 = normalized path length information to each node in the tree
    % term3 = distance from what would be the new node to the goal position
    % result_n=   a1    * dist(p_samp,p_n)   -  a2    *sum(path_info_to_n)/length    +  a3    *   dist(p_new,p_goal) where p_new = p_n+epsilon*vector_to(p_samp)
    qualities = alpha(1)*sqrt(sum(diff.^2,2))-alpha(2)*path_info(:,2)./path_info(:,1)+alpha(3)*(sqrt(sum(((pos(:,1:2)+epsilon*(diff)/norm(diff))-rep_goal).^2,2)));
    [mq, k] = min(qualities);
%     k = dsearchn(pos, v_samp);
    
    samp_dist = sqrt((v_samp(1)-pos(k,1))^2+(v_samp(2)-pos(k,2))^2);
    
    v_new = [0,0];
%     if(samp_dist < epsilon)
%         v_new = v_samp;
%     else
        vec = v_samp-pos(k,:);
        if(~any(vec))
            continue
        end
        vec = vec/norm(vec);
        v_new = round(pos(k,:)+epsilon*vec);
%     end

    % collision checking
    in = 0;
    if(map(v_new(2),v_new(1)) <= threshold)
        in = 1;
    end
    for s = 0:0.1:epsilon;
        step_point = round(pos(k,:)+s*vec);
        if(map(step_point(2), step_point(1)) <= threshold)
            in = 1;
        end
    end
%     for j = 1:length(polys)
%         in = inpolygon(v_new(1),v_new(2), polys{j}(1,:), polys{j}(2,:));
%         if(in > 0)
%             break
%         end
%         [xi, yi] = polyxpoly([pos(k,1);v_new(1)],[pos(k,2);v_new(2)],polys{j}(1,:), polys{j}(2,:));
%         if(~isempty(xi))
%             in = 1;
%             break
%         end
%     end
    if(in > 0)
        continue          
    else        
        nodes(end+1,:) = [k, v_new];
        path = [];
        node = size(nodes,1);
        path = [node];
        while node ~= 0
            path = [path nodes(node,1)];
            node = nodes(node,1);
        end
        path_length = size(path,2)-1;
        path_info(end+1,:) = [path_length, path_info(k,2)+score_map(v_new(2),v_new(1))];
        goal_dist = sqrt((goalpos(1)-v_new(1))^2+(goalpos(2)-v_new(2))^2);
        if(goal_dist <= capture_radius)
            nodes(end+1, :) = [length(nodes), goalpos];
            found_path = 1;
            break
        end
    end
end

%% Find the path
path = [];
if(found_path == 1)
    node = length(nodes);
    path = [node];
    while node ~= 0
        path = [path nodes(node,1)];
        node = nodes(node,1);
    end
end
path_length = length(path)-1

%% Plot the space, tree, and path
imshow(color_map)
% axis('equal')
% xlim(x_lim)
% ylim(y_lim)
hold on
% for i = 1:length(polys)
%     fill(polys{i}(1,:), polys{i}(2,:), 'b')
% end
scatter_nodes = nodes;
scatter_nodes(path(1:end-1)) = [];
scatter(scatter_nodes(:,2),scatter_nodes(:,3),'.','MarkerEdgeColor','k')
for i = 1:length(nodes(:,1))
    par_idx = nodes(i,1);
    if(par_idx == 0)
        continue
    end
    v_new = nodes(i,2:3);
    v_par = nodes(par_idx,2:3);
    if(ismember(i, path))
        line([v_new(1);v_par(1)],[v_new(2);v_par(2)],'Color',[0.91, 0.466, 0.133], 'LineWidth',1)
    else
        line([v_new(1);v_par(1)],[v_new(2);v_par(2)],'Color','k')
    end
end
scatter(nodes(path(1:end-1),2), nodes(path(1:end-1),3), 'o', 'MarkerEdgeColor', [0.91, 0.466, 0.133], 'MarkerFaceColor', [0.91, 0.466, 0.133])
scatter(startpos(1), startpos(2), 'o', 'MarkerFaceColor', 'b')
scatter(goalpos(1), goalpos(2), 'o', 'MarkerFaceColor', 'g')
hold off
