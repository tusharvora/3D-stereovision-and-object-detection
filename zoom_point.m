%% Zoom_ point function

% This function takes image as an input and provide x and y cordinate of...
  ... points selected in the u and v vector.      

function [ u,v ] = zoom_point( image  )
imagesc(image)
axis('equal')
u = []; v = [];
while 0<1
    [x,y,b] = ginput(1); 
    if isempty(b) 
        break;
    elseif b==91
        ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);
        axis([x-width/2 x+width/2 y-height/2 y+height/2]);
        zoom(1/2);
    elseif b==93
        ax = axis; width=ax(2)-ax(1); height=ax(4)-ax(3);
        axis([x-width/2 x+width/2 y-height/2 y+height/2]);
        zoom(2);    
    else
        u=[u;x];
        v=[v;y];
        
    end
end

end

