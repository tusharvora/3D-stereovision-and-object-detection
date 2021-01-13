%% epipolar_line_objectpoints

% This function takes image , the bounding box x limits and the normal...
% ...vector as an input and provide epipolar lines inside the bounding box of...
% ...the object in order to reduce the computational time for stereo matching


function [x_ax,y_ax] = epipolar_line_objectpoints(image,x_min,x_max ,a)

figure();

imagesc(image)
axis('equal')
axis([0 6000 0 4000])
hold on;
    x_ax=(floor(x_min)-50):(floor(x_max)+floor(x_min)+50);
    y_ax=(-(a(1)/a(2)))*x_ax +(-(a(3)/a(2)));
    
    plot(x_ax,y_ax,'color','b');
    xlim auto;
    ylim auto;
end

