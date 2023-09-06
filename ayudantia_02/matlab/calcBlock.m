function [vertices, faces] = calcBlock(r, R, Lx, Ly, Lz)
% Return vertices and faces of a block geometry
    vertices_0 = [
    -Lx/2, -Ly/2,     0;  % #1
    -Lx/2,  Ly/2,     0;  % #2
     Lx/2,     0,     0;  % #3
    -Lx/2, -Ly/2,  Lz/2;  % #4
    -Lx/2,  Ly/2,  Lz/2;  % #5
     Lx/2,     0,  Lz/2]; % #6
    
    vertices = r' + vertices_0*R';
    
    faces = [
    1, 2, 3  1;  % #1
    1, 2, 5, 4;  % #2
    1, 3, 6, 4;  % #3
    2, 3, 6, 5;  % #4
    4, 5, 6  4]; % #5

end