%% blob_object_detection funtion

% This fucntion takes image ,threshold values as an input and provide ...
%... Boundind boxes/object detection of Hammer and screw driver
% Hammer and screw driver automatic selected points are also the  ouput.

function [sdbboxes1,hmbboxes1,hasm_top,hasm_bottom,scs_top,scs_bottom ] = blob_object_detection( image,thresh,d_size,sd_high)
%% Reading image 
A=image;

%% Segmenting Red,Blue,Green plane of image
rmat=A(:,:,1);  % Red plane 
gmat=A(:,:,2);  % Green plane
bmat=A(:,:,3);  % Blue plane
%% Plotting R,G,B planes and original image
figure(1);
subplot(2,2,1),imshow(rmat);title('Red plane');
subplot(2,2,2),imshow(gmat);title('Green plane');
subplot(2,2,3),imshow(bmat);title('Blue plane');
subplot(2,2,4),imshow(A);title('original image');

%% Converting each plane segmented into binary images based on threshold gray value

lr=0.3;lg=0.2; % threshhold binary level of R,G,B plane
%lb=0.1;
Br=im2bw(rmat,lr);Bg=im2bw(gmat,lg);Bb=im2bw(bmat,thresh); % Binary image planes

%% Neglecting green plane and inverting the blue plane and the plotting

Bsum=(Br&(~Bb));
figure(2);
subplot(2,2,1),imshow(Br);title('Red plane');
subplot(2,2,2),imshow(Bg);title('Green plane');
subplot(2,2,3),imshow(Bb);title('Blue plane');
subplot(2,2,4),imshow(Bsum);title('Combined image (Green plane neglected and Blue plane inverted');

%% Applying morphological functions (Erotion and Dialation) to get rid of noise in binary plane

se=strel('disk',d_size); % Disk of size 8 will clear noise
afterOpening=imopen(Bsum,se);figure();%Morphological open function performs erosion follwed by dialation
imshow(afterOpening);
afterClosing=imclose(afterOpening,se);figure();% Morphological close function performs dialation follwed by erosion
imshow(afterClosing);
afterClosing1=imclearborder(~afterClosing);figure();imshow(afterClosing1);% Clears noise atttached at the boundaries of the image.

% Applying again to filter more noise

se1=strel('disk',d_size);
afterOpening2=imopen(afterClosing1,se1);figure();
imshow(afterOpening2);
afterClosing2=imclose(afterOpening2,se1);figure();
imshow(afterClosing2);

%% Labeling/seperating  each blob by using bw label function

[l,num]=bwlabel(afterClosing2,8); % bwlabel function assign each object an different gray value for differentiating object
im1=(l==5); % plotting only one blobobject for testing
figure();imshow(im1);

%% Measuring the area of each blob to identify hammer and screw driver

q=regionprops(l,'Area'); % regionprops function identified the area /no of pixels the blob contains
blobMeasurements=struct2cell(q); % converting structured varirable to cell

%% Classifying objects according to the  pixels areas.

for i=1:num
    
    if blobMeasurements{i}<7000
        continue
    end
    % Screw Driver detection 
    
    if blobMeasurements{i}>50000 && blobMeasurements{i}<sd_high
       sd=(l==i);
       sdbboxes=regionprops(sd,'BoundingBox'); % regionprops (BoundingBox) provides the cordinated of the box
       sdep=regionprops(sd,'Extrema');      % regionprops (Extrema) provides the extreme 8 points of the blob
       sdbboxes1=sdbboxes.BoundingBox;
       figure();imshow(sd);title('screw driver') ;
    % Hammer detection
    
    elseif (blobMeasurements{i}>112000 && blobMeasurements{i}<800000)
        hm=(l==i);
        hmbboxes=regionprops(hm,'BoundingBox'); % regionprops (BoundingBox) provides the cordinated of the box
        hmep=regionprops(hm,'Extrema');         % regionprops (Extrema) provides the extreme 8 points of the blob

        hmbboxes1=hmbboxes.BoundingBox;         
        figure();imshow(hm);title('Hammer') ;
    % Scale/world refrence frame detection  
    
%     elseif blobMeasurements{i}>800000 
%         sc=(l==i);
%         figure();imshow(sc);title('scale');
%        scbboxes=regionprops(sc,'BoundingBox');
%        scep=regionprops(sc,'Extrema');
%        scbboxes1=scbboxes.BoundingBox;

     % coins detection    
%     elseif blobMeasurements{i}>7000 && blobMeasurements{i}<40000
%         co=(l==i);
%         figure();imshow(co);title('coin') 
    end
end
%% Plotting bounding box of Hammer and Screw Driver and selecting automatic  points 

% Hammer head points and tip points segregation

has=struct2cell(hmep); % Converting sturcture to cell variable
%sac=struct2cell(scep); 
hasm=cell2mat(has);    % converting cell to matrix variable
%sasm=cell2mat(sac);

hasmx=hasm(:,1);hasmy=hasm(:,2);
hasmx_top_leftx=(hasmx(1)+hasmx(2))/2;hasmy_top_lefty=(hasmy(1)+hasmy(2))/2;hasm_top_left=[hasmx_top_leftx,hasmy_top_lefty];
hasmx_top_rightx=(hasmx(7)+hasmx(8))/2;hasmy_top_righty=(hasmy(7)+hasmy(8))/2;hasm_top_right=[hasmx_top_rightx,hasmy_top_righty];
hasmx_bottom_leftx=(hasmx(3)+hasmx(4))/2;hasmy_bottom_lefty=(hasmy(3)+hasmy(4))/2;hasm_bottom_left=[hasmx_bottom_leftx,hasmy_bottom_lefty];
hasmx_bottom_rightx=(hasmx(5)+hasmx(6))/2;hasmy_bottom_righty=(hasmy(5)+hasmy(6))/2;hasm_bottom_right=[hasmx_bottom_rightx,hasmy_bottom_righty];

% Based on the distance the tip and head is segregrated
hasm_top_dist=pdist([hasm_top_left;hasm_top_right],'euclidean');

hasm_bottom_dist=pdist([hasm_bottom_left;hasm_bottom_right],'euclidean');

if hasm_top_dist<hasm_bottom_dist

   hasm_top=(hasm_top_left+hasm_top_right)/2;
   hasm_bottom=[hasm_bottom_left;hasm_bottom_right];
else
    
   hasm_bottom=(hasm_bottom_left+hasm_bottom_right)/2;
   hasm_top=[hasm_top_left;hasm_top_right];
end

% Screw driver head points and tip points segregation

scs=struct2cell(sdep); % Converting sturcture to cell variable
scsm=cell2mat(scs);    % converting cell to matrix variable
scsx=scsm(:,1);scsy=scsm(:,2);

scsx_top_leftx=(scsx(1)+scsx(2))/2;scsy_top_lefty=(scsy(1)+scsy(2))/2;scs_top_left=[scsx_top_leftx,scsy_top_lefty];
scsx_top_rightx=(scsx(7)+scsx(8))/2;scsy_top_righty=(scsy(7)+scsy(8))/2;scs_top_right=[scsx_top_rightx,scsy_top_righty];
scsx_bottom_leftx=(scsx(3)+scsx(4))/2;scsy_bottom_lefty=(scsy(3)+scsy(4))/2;scs_bottom_left=[scsx_bottom_leftx,scsy_bottom_lefty];
%scsx_bottom_rightx=(scsx(5)+scsx(6))/2;scsy_bottom_righty=(scsy(5)+scsy(6))/2;scs_bottom_right=[scsx_bottom_rightx,scsy_bottom_righty];

scs_top=[scs_top_left;scs_top_right]; % Screw driver head points
scs_bottom=scs_bottom_left; % Screw Driver tip points

% Plotting the bounding box and points

figure();

imagesc(A)
axis('equal')
axis([0 6000 0 4000])
hold on 

% Screw Driver

rectangle('position',[sdbboxes1(1),sdbboxes1(2),sdbboxes1(3),sdbboxes1(4)],'Edgecolor','r','Linewidth',2,'Tag','Screw Driver')

plot(scsx,scsy,'+','markersize',15,'linewidth',2,'color','g');
plot([scs_top_left(1),scs_top_right(1)],[scs_top_left(2),scs_top_right(2)],'+','markersize',15,'linewidth',2,'color','r');
plot( scs_bottom(1),scs_bottom(2),'+','markersize',15,'linewidth',2,'color','r');

% Hammer

rectangle('position',[hmbboxes1(1),hmbboxes1(2),hmbboxes1(3),hmbboxes1(4)],'Edgecolor','g','Linewidth',2)

 plot(hasmx,hasmy,'+','markersize',15,'linewidth',2,'color','b');
if hasm_top_dist < hasm_bottom_dist
    plot([hasm_bottom_left(1),hasm_bottom_right(1)],[hasm_bottom_left(2),hasm_bottom_right(2)],'+','markersize',15,'linewidth',2,'color','r');
    plot( hasm_top(1),hasm_top(2),'+','markersize',15,'linewidth',2,'color','r');
else
    plot([hasm_top_left(1),hasm_top_right(1)],[hasm_top_left(2),hasm_top_right(2)],'+','markersize',15,'linewidth',2,'color','r');
    plot( hasm_bottom(1),hasm_bottom(2),'+','markersize',15,'linewidth',2,'color','r');
end

hold off

end

