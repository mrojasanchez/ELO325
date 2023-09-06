function vertices = calcVert(root_pos, root_orient, timesteps, Lx, Ly, Lz)
    vertices = zeros(6,3,length(timesteps));
    for i = 1:length(timesteps)
        r = root_pos(i,:)';
        R = eulerXYZ(root_orient(i,1), root_orient(i,2), root_orient(i,3));
        [vertices(:,:,i), ~] = calcBlock(r, R, Lx, Ly, Lz);
    end
end
