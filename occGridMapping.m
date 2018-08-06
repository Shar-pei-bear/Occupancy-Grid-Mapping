function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the initial map size in pixels
myMap = zeros(param.size);

ix_occ = param.resol*(pose(1,:) + ranges.*cos(scanAngles+pose(3,:)));
iy_occ = param.resol*(pose(2,:) - ranges.*sin(scanAngles+pose(3,:)));

% Find grids hit by the rays (in the gird map coordinate)
ix_occ_grid = ceil(ix_occ);
iy_occ_grid = ceil(iy_occ);

ix_ori = pose(1,:)*param.resol;
iy_ori = pose(2,:)*param.resol;

ix_ori_grid = ceil(ix_ori);
iy_ori_grid = ceil(iy_ori);

N = size(pose,2);
M = size(ranges,1);
for j = 1:N % for each time,
%     % Find occupied-measurement cells and free-measurement cells
    free = [];
    for k = 1:M
    [freex,freey] = linedrawing(ix_occ(k,j),iy_occ(k,j),ix_occ_grid(k,j),iy_occ_grid(k,j),...
        ix_ori(j),iy_ori(j),ix_ori_grid(j),iy_ori_grid(j));
    free = [free,[freex;freey]];
    end
    free = free';
    obst = [ix_occ_grid(:,j),iy_occ_grid(:,j)];
    
    free_union = unique(free,'rows');
    obst_union = unique(obst,'rows');
    
    free_ind = sub2ind(param.size, free_union(:,1),free_union(:,2));
    obst_ind = sub2ind(param.size, obst_union(:,1),obst_union(:,2));
    
%     % Update the log-odds
    myMap(free_ind) = myMap(free_ind) - param.lo_free;
    myMap(obst_ind) = myMap(obst_ind) + param.lo_occ;
%     % Saturate the log-odd values
    myMap(myMap > param.lo_max) = param.lo_max;
    myMap(myMap < param.lo_min) = param.lo_min;
end

end

