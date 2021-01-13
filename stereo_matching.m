%% stereo_matching

% This function takes image , x adn y cordinate of epipolar lines,bounding...
% ...box cordinates, Template points cordinates and Template width of point to...
%... be stereo matched usind Sum of square diffence method 
% Output of this function is the minimized cost function output which the...
%...stereomatched points in the other image

function D  = stereo_matching( image,x_2,y_2,box,T1,tem)

     new_min=100000;locax=1;
for ix=1:size(x_2,2)
    %if (y_2(ix)<(box(2)-50)|| y_2(ix)>(box(4)+50))&&y_2(ix)<0&&x_2(ix)<0
   %if (floor(y_2(ix))<floor(box(2)-100)|| floor(y_2(ix))>floor(box(4)+100))
   if (floor(y_2(ix))<floor(box(2))|| floor(y_2(ix))>(floor(box(4))+floor(box(2)))) % points along the epipolar lines
   %if (floor(y_2(ix))>4000 || floor(y_2(ix))<0||floor(x_2(ix))>6000 || floor(x_2(ix))<0)
      
      continue  
   else
       
       % Image 2 point template created as of same size for intensity matching
        hm_points_temp=image((floor(y_2(ix)-tem):floor(y_2(ix)+tem)),(floor(x_2(ix)-tem):floor(x_2(ix)+tem)));  
       %  sum of square Intensity difference of point in Image 1 and the point along epipolar line in the other image
        A=(T1-hm_points_temp).^2; 
        B=sum(sum(A)); % sum of all points inside templates
         if B<new_min % Finding Minimum cost function
           new_min=B;
           locax=ix;
          end   
    end
end
D=locax; % Minimized cost function output / Stereo matched point
end
