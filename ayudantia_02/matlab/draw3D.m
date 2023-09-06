function draw3D(position, angles, t, xyz_goal)
    % Block spec
    Lx = 0.30;
    Ly = 0.20;
    Lz = 0.05;

    % Initial vertices and faces of robot
    r_ini = position(1,:)';
    R_ini = eulerXYZ(angles(1,1), angles(1,2), angles(1,3));
    [vertices_ini, faces_ini] = calcBlock(r_ini, R_ini, Lx, Ly, Lz);
    
    % Initial vertices and face of floor
    [vertices_ini_plane, faces_ini_plane] = calcPlane([0;0;0], eulerXYZ(0,0,0), 10, 10, 0);
    
    % unpack xyz_goal and compute index
    x_goal = xyz_goal(1,:);
    y_goal = xyz_goal(2,:);
    z_goal = xyz_goal(3,:);
    idx = 1:length(x_goal);

    % Draw initial figure
    figure(1);
    h = patch('Faces', faces_ini, 'Vertices', vertices_ini, 'FaceColor', 'y');
    hh = patch('Faces', faces_ini_plane, 'Vertices', vertices_ini_plane, 'FaceColor', 'w');
    hhh = patch(x_goal,y_goal,0,'Marker','.');
    
    % Axes settings
    xlabel('x'); ylabel('y'); zlabel('z');
    axis vis3d equal;
    view(-37.5,30);
    campos([-13.0585  -17.5672   23.1027])
    camlight;
    grid on;
    xlim([-3,3]);
    ylim([-1,5]);
    zlim([-1,1]);
    
    % Compute vertices for each timestep
    vertices = calcVert(position, angles, t, Lx, Ly, Lz);
    
    % Animation Loop
    set(hh, 'Vertices', vertices_ini_plane(:,:,1));
    set(hhh);
    text(x_goal,y_goal,z_goal,[num2cell(idx)],'HorizontalAlignment','left','FontSize',8);
    
    for i = 1:length(t)
        set(h, 'Vertices', vertices(:,:,i));
        drawnow;
    end
end

