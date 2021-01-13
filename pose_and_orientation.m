%% pose_and_orientation

% This function takes three world cordinates points ( for eg Hammer two head pooints and hammer tip point)as an input 
% Provides Origin of object,unit vector X-axis,unit vector Y-axis,unit vector Z-axis and length of object as an output.

function [ d,DA_norm,DC_norm,ZD_norm,dis ] = pose_and_orientation( a,b,c )

syms t
% A, B object top points and C is the bottom points and D is the perpendicular intersecting point from point C To line AB.
AB=b-a; 
r=a+t*AB; % line equation
DC=c-r;   
SD=DC(1)*AB(1)+DC(2)*AB(2)+DC(3)*AB(3);
t=solve(SD==0,t);
t=vpa(t);
d=a+t*AB; % intersecting point D lies on the line AB
DA=d-a;
DC=c-d;
AC=c-a;
dis=norm(cross(AB,AC)/norm(AB)); % Length of object

ZD=cross(DA,DC);
DC_norm=DC/norm(DC); % Y- axis od object
DA_norm=DA/norm(DA); % X- axis of object
ZD_norm=ZD/norm(ZD); % Z- axis of object

end

