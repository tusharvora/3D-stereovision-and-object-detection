%% Clearing and closing everything

clear all
close all

%% Reading Images

image_Matrix1 = imread('image5','jpg'); % Image 1 reading
image_Matrix2 = imread('image4','jpg'); % Image 2 reading
gray_mat1=rgb2gray(image_Matrix1);gray_doub1=double(gray_mat1); % Converting image 1 into graysacle
gray_mat2=rgb2gray(image_Matrix2);gray_doub2=double(gray_mat2); % Converting image 2 into graysacle

%% Calculating Fundamental matrix and plotiing epipolar lines

% Image 1 and 2  selecting test points

npoints=9; % Number of points to be selected
[u1,v1]=zoom_point(image_Matrix1); %  Calling zoom_point function for selecting points with zooming in image 1
[u2,v2]=zoom_point(image_Matrix2); %  Calling zoom_point function for selecting points with zooming in image 1

% Define the point vectors

tic % calculating run time
x = ones(npoints,1); % column vector with values one
p1=[u1,v1,x]; % point vectors 1
p2=[u2,v2,x]; % Point vectors 2

% Calculating the centroids of the test points for Image 1 and 2

cx1=mean(u1(1:(npoints-1))); cy1=mean(v1(1:(npoints-1)));% Image 1 centroid point cordinates
cx2 = mean(u2(1:(npoints-1)));cy2 = mean(v2(1:(npoints-1)));% Image 2 centroid point cordinates

% Calculating the normalization scale factors for Image 1 and 2

sf1=0;sf2=0;
for i=1:(npoints-1)
    a=((u1(i)-cx1)^2+(v1(i)-cy1)^2)^(0.5);
    sf1=sf1+a;
    b=((u2(i)-cx2)^2+(v2(i)-cy2)^2)^(0.5);
    sf2=sf2+b;
end

s1=(sqrt(2))*8/sf1; % scale factor of image 1
s2=(sqrt(2))*8/sf2; % scale factor of image 2

% Normalization matrices

norm_1=[s1,0,-s1*cx1;0,s1,-s1*cy1;0,0,1];
norm_2=[s2,0,-s2*cx2;0,s2,-s2*cy2;0,0,1];

% Normalizing the point vectors and extracting its coordinates

p1_nor=p1*(norm_1)';p2_nor=p2*(norm_2)';

% Computing the normalized Y matrix

Yn=zeros(npoints-1,9);
for i=1:(npoints-1)
    Yn(i,:)= [p1_nor(i,1)*p2_nor(i,1),p1_nor(i,1)*p2_nor(i,2),p1_nor(i,1)*p2_nor(i,3),p1_nor(i,2)*p2_nor(i,1),...
    p1_nor(i,2)*p2_nor(i,2),p1_nor(i,2)*p2_nor(i,3),p1_nor(i,3)*p2_nor(i,1),p1_nor(i,3)*p2_nor(i,2),p1_nor(i,3)*p2_nor(i,3)];
end

% Solving homogenous equation by SVD method to get F matrix elements

[Uy,Sy,Vy] = svd(Yn);

% Defining normalized F based on the last column of Vy 

F_no=Vy(:,end);
F_nor=[F_no(1),F_no(2),F_no(3);F_no(4),F_no(5),F_no(6);F_no(7),F_no(8),F_no(9)];
 
% Computing the SVD of the normalized F matrix

[Uf,Sf,Vf] = svd(F_nor);

% Zero the smallest singular value (Sf) of F and recomputed the normalized F matrix

Sf(3,3)=0;
F_nor_rec=Uf*Sf*Vf';

% Un-normalizing the recomputed F matrix

F_unorm=(norm_1)'* F_nor_rec * norm_2;

% Determine the normal vectors, a1 and a2, for Images 1 and 2

a1=(F_unorm*(p2)')';a2=(F_unorm'*(p1)')'; at=(F_unorm'*[1490,2010,1]')';

% plotting epipolar lines

[x1,y1] = plotting_epipolar_lines( image_Matrix1,npoints,u1,v1,a1);% Calling fuction to plot epipolar line of image 1
[x2,y2] = plotting_epipolar_lines( image_Matrix2,npoints,u2,v2,a2);% Calling fuction to plot epipolar line of image 2
[x3,y3] = plotting_epipolar_lines( image_Matrix2,1,1490,2010,at);% image 2 test point epipolar line

%% Calibrating the camera

% Known world origin

% Lab Test images world cordinate
%points=[0,0,0;45,214,0;94,461,0;168,134,0;234,364,0;485,160,240;570,270,240;670,240,240;680,330,240];

% Final images world cordinate
points=[0,0,0;47,429,0;245,346,0;350,206,0;106,112,0;228,-140,60;112,-175,60;84,-290,100;220,-360,100];
x_points=points(:,1);y_points=points(:,2);z_points=points(:,3);
a1=zeros(npoints-1,12);b1=zeros(npoints-1,12);a2=zeros(npoints-1,12);b2=zeros(npoints-1,12);
A1=[];A2=[];
for i=1:(npoints-1)
    a1(i,:)=[x_points(i),y_points(i),z_points(i),1,0,0,0,0,(-(u1(i))).*x_points(i),(-(u1(i))).*y_points(i),(-(u1(i))).*z_points(i),(-(u1(i)))];
    b1(i,:)=[0,0,0,0,x_points(i),y_points(i),z_points(i),1,(-(v1(i))).*x_points(i),(-(v1(i))).*y_points(i),(-(v1(i))).*z_points(i),(-(v1(i)))];
    
    a2(i,:)=[x_points(i),y_points(i),z_points(i),1,0,0,0,0,(-(u2(i))).*x_points(i),(-(u2(i))).*y_points(i),(-(u2(i))).*z_points(i),(-(u2(i)))];
    b2(i,:)=[0,0,0,0,x_points(i),y_points(i),z_points(i),1,(-(v2(i))).*x_points(i),(-(v2(i))).*y_points(i),(-(v2(i))).*z_points(i),(-(v2(i)))];
    B1=[ a1(i,:); b1(i,:)];B2=[ a2(i,:); b2(i,:)];
  A1=[A1;B1];A2=[A2;B2];
  
end

% solving A matrix to get values of m by SVD

[Ua1,Sa1,Va1] = svd(A1);[Ua2,Sa2,Va2] = svd(A2);

% M (intrinsic and extrinsic properties) elements and matrix

M1= vec2mat(Va1(:,end),4);M2= vec2mat(Va2(:,end),4);

% 8 th coin as a test coin to check M matrix

tp=[points(1,:),1]';
x=M1*tp;y=M2*tp;
x=x./x(3);y=y./y(3);

% Decomposing PPM to obtain intrinsic matrix,Rotaion and origin of cameras

[ K1 , R1 , o1 ] = DecomposePpm(M1); % Calling Decompose function for Camera 1
[ K2 , R2 , o2 ] = DecomposePpm(M2); % Calling Decompose function for Camera 2

% Plotting 3d world cordinates of origin and the points selected

world_ax=[0,0,0;100,0,0;0,100,0;0,0,100];
figure();
hold on;
plot3(x_points(1:npoints),y_points(1:npoints),z_points(1:npoints),'+','markersize',15,'linewidth',2,'color','r');
plot3(o1(1),o1(2),o1(3),'+','markersize',15,'linewidth',2,'color','b');
plot3(o2(1),o2(2),o2(3),'+','markersize',15,'linewidth',2,'color','b');

% plotting the axes pf the camera origin in world cordinates

figure()
hold on
grid on
title('Camera 1 and 2')
xlabel('x')
ylabel('y')
zlabel('z')

quiver3(o1(1),o1(2),o1(3),R1(1,1),R1(2,1),R1(3,1),100,'r') % camera 1 x -axis
quiver3(o1(1),o1(2),o1(3),R1(1,2),R1(2,2),R1(3,2),100,'g') % camera 2 y -axis
quiver3(o1(1),o1(2),o1(3),R1(1,3),R1(2,3),R1(3,3),100,'b') % camera 3 z -axis

quiver3(o2(1),o2(2),o2(3),R2(1,1),R2(2,1),R2(3,1),100,'r') % camera 1 x -axis
quiver3(o2(1),o2(2),o2(3),R2(1,2),R2(2,2),R2(3,2),100,'g') % camera 2 y -axis
quiver3(o2(1),o2(2),o2(3),R2(1,3),R2(2,3),R2(3,3),100,'b') % camera 3 z -axis

hold off

%% Automatic Object Detection and object points detection Image one and two 

thresh_2=0.08;d_size_2=5;sd_high_2=80000;
thresh_1=0.1;d_size_1=10;sd_high_1=70000;

% Calling blob_object_detection for Automatic object and object points detection for image 1 

[sdbboxes1,hmbboxes1,hasm_top1,hasm_bottom1,scs_top1,scs_bottom1] = blob_object_detection(image_Matrix1,thresh_1,d_size_1,sd_high_1);

% Calling blob_object_detection for Automatic object and object points detection for image 2

[sdbboxes2,hmbboxes2,hasm_top2,hasm_bottom2,scs_top2,scs_bottom2] = blob_object_detection(image_Matrix2,thresh_2,d_size_2,sd_high_2); 

 
 %% Epipolar lines of object points selected automatically for stereo matching 

 % Epipolar line for the hammer points
 
 hm_points1=[hasm_top1;hasm_bottom1]; hm_points2=[hasm_top2;hasm_bottom2]; % hammer top and bottom points of image 1
 hm_ones= ones(size( hm_points1,1),1);
 hammer_points_1=[hm_points1,hm_ones]; hammer_points_2=[hm_points2,hm_ones]; % Hammer top and bottom points vector of image 1 
 hammer_epi2=(F_unorm'*(hammer_points_1)')'; % Determining the normal vector of hammer points by multiplying with fundamental matrix
 
% Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of hammer top/head point 1

[hx21,hy21] = epipolar_line_objectpoints(image_Matrix2,hmbboxes2(1),hmbboxes2(3),hammer_epi2(1,:));
%  Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of hammer top/head point 2

[hx22,hy22] = epipolar_line_objectpoints(image_Matrix2,hmbboxes2(1),hmbboxes2(3),hammer_epi2(2,:));
%  Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of hammer bottom/tip point

[hx23,hy23] = epipolar_line_objectpoints(image_Matrix2,hmbboxes2(1),hmbboxes2(3),hammer_epi2(3,:));


% Epipolar line for the screw driver points

 sd_points1=[scs_top1;scs_bottom1]; sd_points2=[scs_top2;scs_bottom2]; % Screw Driver top and bottom points of image 1
 sd_ones= ones(size( sd_points1,1),1);
 screw_points_1=[sd_points1,sd_ones]; screw_points_2=[sd_points2,sd_ones]; % Screw Driver top and bottom points vector of image 1 
 screw_epi2=(F_unorm'*(screw_points_1)')';  % Determining the normal vector of Screw Driver points by multiplying with fundamental matrix
 
 %  Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of Screw Driver top/head point 1
[sx21,sy21] = epipolar_line_objectpoints(image_Matrix2,sdbboxes2(1),sdbboxes2(3),screw_epi2(1,:));

%  Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of Screw Driver top/head point 2
[sx22,sy22] = epipolar_line_objectpoints(image_Matrix2,sdbboxes2(1),sdbboxes2(3),screw_epi2(2,:));

%  Calling epipolar_line_objectpoints function for plotting epipolar line in image 2 of Screw Driver bottom/tip point 1
[sx23,sy23] = epipolar_line_objectpoints(image_Matrix2,sdbboxes2(1),sdbboxes2(3),screw_epi2(3,:));

%% Stereo matching along epipolar line to find point in image 1 in to image 2

template_width=10; % Template size around the point selected for matching

% Hammer points template of image 1 for matching in  image 2

  hm_points1_temp_1  =gray_doub1((floor(hm_points1(1,2)-template_width):floor(hm_points1(1,2)+template_width)),...
                                (floor(hm_points1(1,1)-template_width):floor(hm_points1(1,1)+template_width)));
  hm_points1_temp_2  =gray_doub1(floor(hm_points1(2,2)+50-template_width):floor(hm_points1(2,2)+50+template_width),...
                                 floor(hm_points1(2,1)+50-template_width):floor(hm_points1(2,1)+50+template_width));
  hm_points1_temp_3  =gray_doub1(floor(hm_points1(3,2)-template_width):floor(hm_points1(3,2)+template_width),...
                                 floor(hm_points1(3,1)-template_width):floor(hm_points1(3,1)+template_width));

% Screw Driver points template of image 1 for matching in  image 2

  sd_points1_temp_1  =gray_doub1((floor(sd_points1(1,2)-template_width):floor(sd_points1(1,2)+template_width)),...
                                 (floor(sd_points1(1,1)-template_width):floor(sd_points1(1,1)+template_width)));
  sd_points1_temp_2  =gray_doub1(floor(sd_points1(2,2)-template_width):floor(sd_points1(2,2)+template_width),...
                                 floor(sd_points1(2,1)-template_width):floor(sd_points1(2,1)+template_width));
  sd_points1_temp_3  =gray_doub1(floor(sd_points1(3,2)-template_width):floor(sd_points1(3,2)+template_width),...
                                 floor(sd_points1(3,1)-template_width):floor(sd_points1(3,1)+template_width));

% Stereo matching the points

% Hammer head and tip points stereo matching

hm1_st_x2=stereo_matching(gray_doub2,hx21,hy21,hmbboxes2,hm_points1_temp_1,template_width); % Calling stereo_matching function to match hammer head point 1
hm2_st_x2=stereo_matching(gray_doub2,hx22,hy22,hmbboxes2,hm_points1_temp_2,template_width); % Calling stereo_matching function to match hammer head point 2
hm3_st_x2=stereo_matching(gray_doub2,hx23,hy23,hmbboxes2,hm_points1_temp_3,template_width); % Calling stereo_matching function to match hammer Tip point 1

% Screw Driver head and tip points stereo matching

sd1_st_x2=stereo_matching(gray_doub2,sx21,sy21,sdbboxes2,sd_points1_temp_1,template_width); % Calling stereo_matching function to match Screw Driver head point 1
sd2_st_x2=stereo_matching(gray_doub2,sx22,sy22,sdbboxes2,sd_points1_temp_2,template_width); % Calling stereo_matching function to match Screw Driver head point 1
sd3_st_x2=stereo_matching(gray_doub2,sx23,sy23,sdbboxes2,sd_points1_temp_3,template_width); % Calling stereo_matching function to match Screw Driver tip point 1

% Plotting Stereo matched points of image 2

figure()
imagesc(image_Matrix2)
hold on
plot(hx21(hm1_st_x2),hy21(hm1_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 
plot(hx22(hm2_st_x2),hy22(hm2_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 
plot(hx23(hm3_st_x2),hy23(hm3_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 

plot(sx21(sd1_st_x2),sy21(sd1_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 
plot(sx22(sd2_st_x2),sy22(sd2_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 
plot(sx23(sd3_st_x2),sy23(sd3_st_x2),'+','markersize',15,'linewidth',2,'color','r'); 
hold off


%% Rectification process/parameters

to = (o2-o1);% Distance between two optical centre of camera 
r1=to/norm(to); % New 'x' axis of image plane 
r21=cross(R1(3,:)',to);r22=cross(R2(3,:)',to);% New 'y' axis rotation component
r31=cross(r1,r21);r32=cross(r1,r22);% New 'z' axis rotation component
R1_new=[r1';r21'/norm(r21);r31'/norm(r31)];R2_new=[r1';r22'/norm(r22);r32'/norm(r32)];% New rotation matrix of camera 1 and 2

K_new=(K1+K2)/2;% New intrinsic property will be mean of the K1,and K2
Q1_old=K1*R1;Q2_old=K2*R2; % Old Q
Q1_new=K_new*R1_new;Q2_new=K_new*R2_new; % New Q
T1=Q1_new*(Q1_old)^(-1);T2=Q2_new*(Q2_old)^(-1); % Transformation matrix of camera 1 and 2 in to rectified plane

%[rectifiedleft, rectifiedright] = rectifyimage(image_Matrix1, image_Matrix2, T1, T2);

%% 3D Reconstruction of points in image 1 and 2

% 3d world cordinarte of manually selected points in both images 1 and 2

P1_new_1=(T1*p1')';P2_new_2=(T2*p2')';
P_world  = reconstruction(o1,to,P1_new_1,P2_new_2 ,R1_new,R2_new,K_new);

% 3d world cordinarte of Automatically selected hammer points in both images 1 and 2

hammer_points_2_ste=[hx21(hm1_st_x2),hy21(hm1_st_x2),1;hx22(hm2_st_x2),hy22(hm2_st_x2),1;hx23(hm3_st_x2),hy23(hm3_st_x2),1];
hammer_points_11=(T1*hammer_points_1')';hammer_points_22_ste=(T2*hammer_points_2_ste')';
hammer_points_22=(T2*hammer_points_2')';

    % Finding Correct hammer points of first image into other image either automatically detected point or stereo matched points based on the Euclidesn distance 

    hd11=pdist([hx21(hm1_st_x2),hy21(hm1_st_x2);hm_points2(1,:)],'euclidean');
    hd12=pdist([hx21(hm2_st_x2),hy21(hm2_st_x2); hm_points2(2,:)],'euclidean');
    P_world_1  = reconstruction(o1,to,hammer_points_11,hammer_points_22_ste ,R1_new,R2_new,K_new);

    if hd11<hd12
    
       Ph_world_2  = reconstruction(o1,to,hammer_points_11,hammer_points_22 ,R1_new,R2_new,K_new);
    else
       hammer_points_22_new=[hammer_points_22(2,:);hammer_points_22(1,:);hammer_points_22(3,:)];
       Ph_world_2  = reconstruction(o1,to,hammer_points_11,hammer_points_22_new ,R1_new,R2_new,K_new);
    end

    Ph_world_2(1,1)= Ph_world_2(1,1)*(-1);Ph_world_2(1,2)= Ph_world_2(1,2)*(-1);

% 3d world cordinarte of Automatically selected screw Driver points in both images 1 and 2

screw_points_2_ste=[sx21(sd1_st_x2),sy21(sd1_st_x2),1;sx22(sd2_st_x2),sy22(sd2_st_x2),1;sx23(sd3_st_x2),sy23(sd3_st_x2),1];
screw_points_11=(T1*screw_points_1')';screw_points_22_ste=(T2*screw_points_2_ste')';
screw_points_22=(T2*screw_points_2')';

Ps_world_1  = reconstruction(o1,to,screw_points_11,screw_points_22_ste ,R1_new,R2_new,K_new); % Reconstruction of stereo matched points

Ps_world_2  = reconstruction(o1,to,screw_points_11,screw_points_22 ,R1_new,R2_new,K_new); % Reconstruction of automatic points selected

p111=(T1*[365,906,1]')';p222=(T2*[554,1762,1]')'; % Reconstruction of manually selected points
Ps_world_4=reconstruction(o1,to,p111,p222 ,R1_new,R2_new,K_new);

%% Pose estimation and orientaion

% HAmmer 

% calling pose_and_orientation function for finding leght of hammer and rotation matrix 
 [ HD,H_DA,H_DC,H_ZD,H_dis ] = pose_and_orientation( Ph_world_2(:,1)',Ph_world_2(:,2)',Ph_world_2(:,3)' ); 
 
 Rot_ham=[H_DA;H_DC;H_ZD]; % Rotation matrix of hammer in terms of wolrd to hammer

% Screw Driver
 
% calling pose_and_orientation function for finding leght of Screw driver and rotation matrix
 [ SD,S_DA,S_DC,S_ZD,S_dis ] = pose_and_orientation( Ps_world_1(:,1)',Ps_world_1(:,2)',Ps_world_1(:,3)' ); 
 
 Rot_scw=[S_DA;S_DC;S_ZD]; % Rotation matrix of Screw driver in terms of wolrd to Screw driver
 
% Plotting the Axis of Camera 1,Camera 2,Hammer and the screw Driver with respect to world.

figure()
hold on
grid on
title('Camera 1')
xlabel('x')
ylabel('y')
zlabel('z')

% Hammer x ,y and z axis respectively

quiver3(HD(1),HD(2),HD(3),Rot_ham(1,1),Rot_ham(2,1),Rot_ham(3,1),100,'r')
quiver3(HD(1),HD(2),HD(3),Rot_ham(1,2),Rot_ham(2,2),Rot_ham(3,2),100,'g')
quiver3(HD(1),HD(2),HD(3),Rot_ham(1,3),Rot_ham(2,3),Rot_ham(3,3),100,'b')

% Screw Driver x,y, and z axis respectively

quiver3(SD(1),SD(2),SD(3),Rot_scw(1,1),Rot_scw(2,1),Rot_scw(3,1),100,'r')
quiver3(SD(1),SD(2),SD(3),Rot_scw(1,2),Rot_scw(2,2),Rot_scw(3,2),100,'g')
quiver3(SD(1),SD(2),SD(3),Rot_scw(1,3),Rot_scw(2,3),Rot_scw(3,3),100,'b')

% Camera 1 x,y, and z axis respectively

quiver3(o1(1),o1(2),o1(3),R1(1,1),R1(2,1),R1(3,1),100,'r')
quiver3(o1(1),o1(2),o1(3),R1(1,2),R1(2,2),R1(3,2),100,'g')
quiver3(o1(1),o1(2),o1(3),R1(1,3),R1(2,3),R1(3,3),100,'b')

%  Camera 2 x,y, and z axis respectively

quiver3(o2(1),o2(2),o2(3),R2(1,1),R2(2,1),R2(3,1),100,'r')
quiver3(o2(1),o2(2),o2(3),R2(1,2),R2(2,2),R2(3,2),100,'g')
quiver3(o2(1),o2(2),o2(3),R2(1,3),R2(2,3),R2(3,3),100,'b')

hold off
 
toc
run_time=toc;