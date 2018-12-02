%==========================================================================
%   DESCRIPTION: Solve shortest path problem with RRT P4
%
%   AUTHOR: jjimenez
%   DATE: 11/22/2018
%==========================================================================
 close all; clear all; clc;
%%
%--------------------------------------------------------------------------
%                       Initiaize Arena
%--------------------------------------------------------------------------
max_step = 0.5;
end_criteria = 1; % Norm of the error in position before we say we have acheived in finding a path to the goal

% robot_init = [-10;0];
% robot_goal = [7;0];
robot_init = [-10;0];
robot_goal = [9;0];
robot = robot_init;

[i_x,i_y] = meshgrid(-10:0.5:10, -10:0.5:10);
information = i_y.*sin(i_x) - i_x .* cos(i_y);
%information = sinc(sqrt((i_x/pi).^2+(i_y/pi).^2));

% information = 10.*information;
% information(13:30,13:30) =-1.* abs(information(13:30,13:30));
% information(12:31,12:31) =-1.* abs(information(12:31,12:31));
%information(10:33,1:33) =-1.* abs(information(10:33,1:33));
%information = information - max(information(:));
figure;
surf(i_x, i_y, information)

% [~, tix] = max(i_x(1,find(i_x(1,:)<= tree{i}.p(1))));
% [~, tiy] =  max(i_y(find(i_y(:,1)<= 3),1))
tree = {};
tree = append_node(tree,robot_init,0, 0);
%%
i = 1;
ii = 0;
add_flag = 0;
closest_node = 1;
%figure('pos', [2000 100 1200 900]);
figure;
imagesc(i_x(1,:),i_y(:,1), information); hold on;
plot(robot(1), robot(2) ,'.r', 'MarkerSize', 15); hold on;
plot(robot_goal(1), robot_goal(2) ,'.g', 'MarkerSize', 15);
% surf(i_x, i_y, information)
 
  while(1)
    q = -10 + (20)*rand(2,1);   % Sample from configuration space
    mindist = 10000;
     for i=1:length(tree)
        %------------------------------------------------------------------
        %           Check minimum distance
        %------------------------------------------------------------------
        if ii == 1
            dist = norm(tree{i}.p);
        else  
            
            a = atan2(q(2)-tree{i}.p(2), q(1)-tree{i}.p(1));
            max_x = max_step*cos(a);
            max_y = max_step*sin(a);
            qpossible = [(tree{i}.p(1) + max_x); (tree{i}.p(2) + max_y)];
            rox = linspace(tree{closest_node}.p(1), qpossible(1), 5);
            roy = linspace(tree{closest_node}.p(2), qpossible(2), 5);
            [~, tix] = max(i_x(1,find(i_x(1,:)<= tree{i}.p(1))));
            [~, tiy] =  max(i_y(find(i_y(:,1)<= tree{i}.p(2)),1));
            sum_of_rewards = 0;
            for j = 1:5
                [~, qpix] = max(i_x(1,find(i_x(1,:)<= rox(j))));
            	[~, qpiy] =  max(i_y(find(i_y(:,1)<= roy(j)),1));
                sum_of_rewards = sum_of_rewards + information(qpiy,qpix);
            end 
             
            
%             dist = norm(q - tree{i}.p) - 0.217*information(tix,tiy);%; - norm(robot_goal - tree{i}.p);
            %dist = norm(q - tree{i}.p) - information(tix,tiy);%- information(qix,qiy);
            %dist = norm(q - tree{i}.p) - 0.2*information(tiy,tix);% + 0.3*norm(robot_goal - tree{i}.p);
%             dist = norm(q - tree{i}.p) - (0.001)*tree{i}.info;% + 0.3*norm(robot_goal - tree{i}.p);
            dist = norm(q-tree{i}.p) - 0.01*sum_of_rewards - 0.1*information(tiy,tix);
        end
        if (dist < mindist)
            mindist = dist;
            closest_node = i;
        end    
     end
    
    %----------------------------------------------------------------------
    %               Take a step in direction of "q"
    %----------------------------------------------------------------------
    a = atan2(q(2)-tree{closest_node}.p(2), q(1)-tree{closest_node}.p(1));
    max_x = max_step*cos(a);
    max_y = max_step*sin(a);
    qnew = [(tree{closest_node}.p(1) + max_x); (tree{closest_node}.p(2) + max_y)];
    if (norm(q-tree{closest_node}.p) <= norm(qnew - tree{closest_node}.p))
        qnew = q;
    end
   
    %----------------------------------------------------------------------
    %               Collision checking
    %----------------------------------------------------------------------

    [~, qix] = max(i_x(1,find(i_x(1,:)<= qnew(1))));
    [~, qiy] =  max(i_y(find(i_y(:,1)<= qnew(2)),1));
    tree = append_node(tree,qnew,closest_node, tree{closest_node}.info + information(qiy,qix));
    i = length(tree);
    robotm = qnew;
    add_flag = 1;
    
    
%     plot(q(1), q(2), '.'); % Plots q_rand but its commented out to unclutter the figure
    if add_flag == 1
        plot([tree{closest_node}.p(1) qnew(1)], [tree{closest_node}.p(2) qnew(2)], 'k');
%         surf(i_x, i_y, information)
        
        add_flag = 0;
    end
    title(['Iterations: ', num2str(ii), ' Maximum Step Size: ', num2str(max_step)]);
    axis([-10 10 -10 10])
    pause(eps)
    
    % Ending condition
    if (norm(qnew - robot_goal)< end_criteria)
            break
    end
    
    ii = ii+1;
end

i = length(tree);
        P(:,1) = tree{i}.p;
        while 1
            i = tree{i}.parent_pointer;
            if i == 0
                figure
                surf(i_x,i_y,information); hold on;
                plot(robot_goal(1), robot_goal(2), '.g', 'MarkerSize', 15);
                plot3(P(1,:), P(2,:), max(information(:)).*ones(1,length(P)),'r', 'LineWidth', 5);
                hold off;
                title('RRT Path')
                axis([-10 10 -10 10])
                return
            end
            P = [tree{i}.p P];
        end
   %}     