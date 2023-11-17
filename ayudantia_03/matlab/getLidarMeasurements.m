function [mean_dist_front, mean_dist_left_front, mean_dist_left_back, mean_dist_right_front, mean_dist_right_back, r_ang, l_ang] = getLidarMeasurements(unpackedsignal,radians)
    
% Input:
%   unpackedsignal: the full 648 points comming from the lidar sensor
%   radians: 684 vector with readings from -120 to 120
% Output:
%   mean_dist_front: average front distance (-5°,5°)
%   mean_dist_left_front, mean_dist_left_back
%   mean_dist_right_front, mean_dist_right_back 
%   r_ang, l_ang: slopes of left and right interpolated lines, used later
%                in the wall following algorithm

    frnt_idx = [328:357];   % -5° to 5° 

    % r_idx = [43:129];
    rb_idx = [43:57];       % -105.2416° to -100.3221°
    rf_idx = [114:129];     % -80.2928° to -75.0220°

    % l_idx = [556:642];
    lf_idx = [556:571];     % 75.0220° to 80.2928°
    lb_idx = [628:642];     % 100.3221° to 105.2416°
    %-------------------
    front_lidar = unpackedsignal(frnt_idx); % -5.0952° to 5.0952°
    
    right_back_lidar = unpackedsignal(rb_idx); 
    right_front_lidar = unpackedsignal(rf_idx); 
    
    left_front_lidar = unpackedsignal(lf_idx); 
    left_back_lidar = unpackedsignal(lb_idx); 
    %-------------------
    front_radians = radians(frnt_idx);
    
    % right_radians = radians(r_idx);
    right_back_radians = radians(rb_idx);
    right_front_radians = radians(rf_idx);

    % left_radians = radians(l_idx);
    left_front_radians = radians(lf_idx);
    left_back_radians = radians(lb_idx);
    %-------------------
    mean_dist_front = mean(front_lidar,"all");
    
    mean_dist_right_front = mean(right_front_lidar,"all");
    mean_dist_right_back = mean(right_back_lidar,"all");

    mean_dist_left_front = mean(left_front_lidar,"all");
    mean_dist_left_back = mean(left_back_lidar,"all");
    %------------------- 
    [x,y] = pol2cart(radians,unpackedsignal);
    
    [x_front,y_front] = pol2cart(front_radians,front_lidar);
    
    [x_right_front,y_right_front] = pol2cart(right_front_radians,right_front_lidar);
    [x_right_back,y_right_back] = pol2cart(right_back_radians,right_back_lidar);
    
    [x_left_front,y_left_front] = pol2cart(left_front_radians,left_front_lidar);
    [x_left_back,y_left_back] = pol2cart(left_back_radians,left_back_lidar);

    %------------------- Fitting curves
    % coeff_front = polyfit(x_front, y_front, 1);
    % xFit_front = x_front; % Option 1 : same number of points as the training set.
    % yFit_front = polyval(coeff_front, xFit_front);

    coeff_right = polyfit([x_right_back x_right_front], [y_right_back y_right_front], 1);
    xFit_right = [x_right_back x_right_front]; % Option 1 : same number of points as the training set.
    yFit_right = polyval(coeff_right, xFit_right);

    % xFit_right_centered = linspace(min(x_right_front)-1,max(x_right_front)+1,10); % Option 2 : lots of points, and not just where the training points are.
    % yFit_right_centered = polyval([coeff_right(1) 0], xFit_right_centered);

    coeff_left = polyfit([x_left_back x_left_front], [y_left_back y_left_front], 1);
    xFit_left = [x_left_back x_left_front]; % Option 1 : same number of points as the training set.
    yFit_left = polyval(coeff_left, xFit_left);

    % xFit_left_centered = linspace(min(x_left_front)-1,max(x_left_front)+1,10); % Option 2 : lots of points, and not just where the training points are.
    % yFit_left_centered = polyval([coeff_left(1) 0], xFit_left_centered);

    r_ang = atand(coeff_right(1));
    l_ang = atand(coeff_left(1));
    % fprintf('frnt_d: %.2f\t lft_fd: %.2f\t lft_bd: %.2f\t rgt_fd: %.2f\t rgt_bd: %.2f\t', mean_dist_front, mean_dist_left_front, mean_dist_left_back, mean_dist_right_front, mean_dist_right_back);
    % fprintf('right_angle: %.2f\tleft_angle: %.2f\n', r_ang, l_ang);

    % Plotting
    p = plot(x,y,'linestyle','none','marker','o');
    p.Marker = ".";
    
    hold on;
    grid on;
    xlim([-3,3]);
    ylim([-3,3]);
    xlabel('x','Interpreter','latex');
    ylabel('y','Interpreter','latex');

    plot(x_front,y_front,'c-', 'LineWidth', 2)
    plot(x_right_front,y_right_front,'m-', 'LineWidth', 2)
    plot(x_right_back,y_right_back,'g-', 'LineWidth', 2)
    plot(x_left_front,y_left_front,'m-', 'LineWidth', 2)
    plot(x_left_back,y_left_back,'g-', 'LineWidth', 2)

    % plot(xFit_front, yFit_front, 'm-', 'LineWidth', 2);
    plot(xFit_right, yFit_right, 'r-', 'LineWidth', 2);
    plot(xFit_left, yFit_left, 'r-', 'LineWidth', 2);
    % plot(xFit_right_centered, yFit_right_centered, 'g-', 'LineWidth', 2);
    % plot(xFit_left_centered, yFit_left_centered, 'r-', 'LineWidth', 2);
    
    drawnow;
    hold off;
end

