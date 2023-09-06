function [vertices, faces] = calcPlane(r, R, Lx, Ly, Lz)
% Return vertices and faces of a block geometry
    vertices_0 = [
    -Lx, -Ly,   0;  % #1
    -Lx,  Ly,   0;  % #2
     Lx,  Ly,   0;  % #3
     Lx, -Ly,  Lz]; % #4
    
    vertices = r' + vertices_0*R';
    
    faces = [
    1, 2, 3  4]; % #1

end
