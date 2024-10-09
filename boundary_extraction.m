clear all;
clc;
close all;

I = imread('A1_Ring.png');
BW = imbinarize(I);
imshow(BW);
BW = 1-BW;
[B,L] = bwboundaries(BW, 'holes');

num_of_boundary = length(B);
boundary = B{1};
boundary2 = B{2};

interval = 1;
plot(boundary(1:interval:end,2),boundary(1:interval:end,1),'r.');

hold on;
plot(boundary2(1:interval:end,2),boundary2(1:interval:end,1), 'g.');

f1 = figure;


num_of_points = size(boundary, 1);

max_y = 0;
pts = [boundary(:, 2), boundary(:, 1)];
% 1324th row is the starting point
% pts = [pts(1324:end, :); pts(1:1323, :)];
qts = zeros(num_of_points, 2);
dist_to_prev = zeros(num_of_points, 1);
dist_accumulative = zeros(num_of_points, 1);
direction = zeros(num_of_points, 2);
degree = zeros(num_of_points, 1);

for i = 1 : num_of_points
    if boundary(i, 1) > max_y
        max_y = boundary(i, 1);
    end
end
pts(:, 2) = max_y - pts(:, 2);

for i = 1 : num_of_points    
    prev_index = mod(i-1-1, num_of_points) + 1;
    next_index = mod(i+1-1, num_of_points) + 1;
    next_next_index = mod(i+2-1, num_of_points) + 1;
    
    
    y0 = pts(prev_index, 2);
    x0 = pts(prev_index,1);
    y1 = pts(i,2);
    x1 = pts(i,1);
    y2 = pts(next_index, 2);
    x2 = pts(next_index, 1);
    y3 = pts(next_next_index, 2);
    x3 = pts(next_next_index, 1);
    
    
    delta_d = norm([x1,y1] - [x0,y0]);
    dist_to_prev(i) = delta_d;
    if i == 1
        dist_accumulative(i) = 0;
    else
        dist_accumulative(i) = dist_accumulative(i-1) + dist_to_prev(i);
    end
    
    direction(i, :) = [x2,y2] - [x1,y1];
    degree = atan2d(direction(i,2), direction(i,1));
    
    qts(i, :) = pts(i, :) + 50 * [cosd(degree - 90), sind(degree - 90)];
end



for i = 1 : num_of_points
    % if mod(i,5) == 0
        plot(pts(i,1), pts(i,2), 'r.');
        hold on;
        % plot(qts(i,1), qts(i,2), 'bo');
        hold on;
    % end
end
title('pts');
axis equal;

dist_accumulative(end)
max(dist_to_prev)
min(dist_to_prev)


% build boundary_copy that doesn't have consecutive points having the same
% x or y value
boundary_copy = [];
i = 1;
while i <= num_of_points
    if i ~= num_of_points && pts(i,1) == pts(i+1,1)
        s = i;
        n = 2;
        t = i + 1;
        while pts(t, 1) == pts(t+1,1)
            t = t+1;
            if t == num_of_points
                break;
            end
        end
        boundary_copy = [boundary_copy; (pts(s, :) + pts(t, :)) / 2];
        %if t - s > 1
        %    boundary_copy = [boundary_copy; pts(s, :); pts(s + floor((t-s)/2), :); pts(t, :)];
        %elseif t - s == 1
        %    boundary_copy = [boundary_copy; pts(s, :); pts(t, :)];
        %end
        i = t + 1;
    elseif i ~= num_of_points && pts(i,2) == pts(i+1,2)
        s = i;
        n = 2;
        t = i + 1;
        while pts(t, 2) == pts(t+1,2)
            t = t+1;
            if t == num_of_points
                break;
            end
        end
        boundary_copy = [boundary_copy; (pts(s, :) + pts(t, :)) / 2];
        %if t - s > 1
        %    boundary_copy = [boundary_copy; pts(s, :); pts(s + floor((t-s)/2), :); pts(t, :)];
        %elseif t - s == 1
        %    boundary_copy = [boundary_copy; pts(s, :); pts(t, :)];
        %end
        i = t + 1;
    else
        boundary_copy = [boundary_copy; pts(i, :)];
        i = i + 1;
    end
end


num = length(boundary_copy(:,1));
pts_2 = [boundary_copy(:, 2), boundary_copy(:, 1)];
qts_2 = zeros(num, 2);
dist_to_prev_2 = zeros(num, 1);
dist_accumulative_2 = zeros(num, 1);
direction_2 = zeros(num, 2);
degree_2 = zeros(num, 1); 
for i = 1 : num    
    prev_index_2 = mod(i-1-1, num) + 1;
    next_index_2 = mod(i+1-1, num) + 1;
    next_next_index_2 = mod(i+2-1, num) + 1;
    
    
    y0 = pts_2(prev_index_2, 2);
    x0 = pts_2(prev_index_2,1);
    y1 = pts_2(i,2);
    x1 = pts_2(i,1);
    y2 = pts_2(next_index_2, 2);
    x2 = pts_2(next_index_2, 1);
    y3 = pts_2(next_next_index_2, 2);
    x3 = pts_2(next_next_index_2, 1);
    
    
    delta_d_2 = norm([x1,y1] - [x0,y0]);
    dist_to_prev_2(i) = delta_d_2;
    if i == 1
        dist_accumulative_2(i) = 0;
    else
        dist_accumulative_2(i) = dist_accumulative_2(i-1) + dist_to_prev_2(i);
    end
    
    direction_2(i, :) = [x2,y2] - [x1,y1];
    degree_2 = atan2d(direction_2(i,2), direction_2(i,1));
    
    qts_2(i, :) = pts_2(i, :) + 10 * [cosd(degree_2 + 90), sind(degree_2 + 90)];
end



figure;
for i = 1:1:size(boundary_copy, 1)
    plot(boundary_copy(i,1), boundary_copy(i,2), 'r.');
    hold on;
    plot(qts_2(i,2), qts_2(i,1), 'g.');
    hold on;
end
title('POINTS ONLY of boundary_copy & qts');
axis equal;

figure;
plot(boundary_copy(1:1:end,1), boundary_copy(1:1:end,2), 'r-');
hold on;
plot(qts_2(1:2:end,2), qts_2(1:2:end,1), 'g-');
title('CURVE of boundary_copy & qts');
axis equal;

