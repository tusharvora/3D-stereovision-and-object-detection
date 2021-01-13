%% reconstruction

% This function takes origin of camera 1, distance between two origin,New rectified points ,New rectified rotation matrix,and New average intrinsic matrix
% Provides final 3d point as an output by traingulation theory

function P_world  = reconstruction(o1,to,P1_new,P2_new,R1_new,R2_new,K_new )
for i=1:size(P1_new,1)
    
p11=(R1_new)'*inv(K_new)*((P1_new(i,:))/(P1_new(i,3)))'; % Image 1 point in rectified plane is converted to wolrd point vector
p22=(R2_new)'*inv(K_new)*((P2_new(i,:))/(P2_new(i,3)))'; % Image 2 point in rectified plane is converted to wolrd point vector

j=cross(p11,p22); % Perpendicular vector from two image point vectors
j1=j/norm(j); % unit vector
P=[p11,p22,j1]; 
ka=inv(P);
ka1=ka*to;

P_world(:,i)=o1+ka1(1,1)*p11+(ka1(3,1)/2)*j1; % final mid point of the two vector perpendicular vector

end

