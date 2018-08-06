clear all;
close all;

load practice.mat 
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement (in meter) at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] pose is 3-by-K array containing the pose of the mobile robot over time. 
%     e.g. pose(:,k) is the [x(meter),y(meter),theta(in radian)] at time index k.

% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 2. Decide the initial map size in pixels

lidar_globa1_x = pose(1,:) + ranges.*cos(scanAngles+pose(3,:));
lidar_globa1_y = pose(2,:) - ranges.*sin(scanAngles+pose(3,:));

param.size(1) = ceil(param.resol*(max(lidar_globa1_x(:))-min(lidar_globa1_x(:))));
param.size(2) = ceil(param.resol*(max(lidar_globa1_y(:))-min(lidar_globa1_y(:))));

% 3. Indicate the origin in pixels
param.offset(1) = (param.size(1)/param.resol - max(lidar_globa1_x(:)) - min(lidar_globa1_x(:)))/2;
param.offset(2) = (param.size(2)/param.resol - max(lidar_globa1_y(:)) - min(lidar_globa1_y(:)))/2;

pose(1:2,:) = pose(1:2,:) + param.offset';
param.origin = ceil(pose(1:2,1)*param.resol);

% 4. Log-odd parameters 
param.lo_occ = 1;
param.lo_free = 0.5; 
param.lo_max = 100;
param.lo_min = -100;

myMap = occGridMapping(ranges, scanAngles, pose, param);

% The final grid map: 
figure,
imagesc(myMap); 
colormap('gray');
