%% plotting_epipolar_lines

% This function takes image ,number of points and normal vector ontained from the fundametal matrix  as an input and provides...
 %... the x and y cordinate of epipolar line as an output.
% This fucntion plots the epipolar line 
    
function [x_ax,y_ax] = plotting_epipolar_lines( image,npoints,u,v,a)

figure();

imagesc(image)
axis('equal')
axis([0 6000 0 4000])
hold on;
%plot(u(1:(npoints-1)),v(1:(npoints-1)),'+','markersize',15,'linewidth',2,'color','r');% Ploting non test points
%plot(u((npoints-1):npoints),v((npoints-1):npoints),'+','markersize',15,'linewidth',2,'color','g');%Ploting  test points

for i=1:npoints
    
 x_ax=0:size(image,2);
    y_ax=(-(a(i,1)/a(i,2)))*x_ax +(-(a(i,3)/a(i,2)));
    
    plot(x_ax,y_ax,'color',[i/10 i/20 i/30]);
    xlim auto;
    ylim auto;
end
end