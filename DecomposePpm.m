%% DecomposePPm function
% This fucntion takes projection matrix as an input and provides intrinsic...
%... matrix, rotation matrix and origin of camera as an output

function [  K , R , o ] = DecomposePpm(M)

% Computes the intrinsic matrix (K), rotation matrix (R), and origin offset (o) for the given perspective projection matrix (M)

[Temp1,Temp2] = qr(flipud(M(:,1:3))');

% Calculate the intrinsic matrix (right-upper-rectangular)
%K = fliplr( flipud( Temp2') );
K = rot90(  Temp2',2 );

% Calculate the rotation matrix (orthonormal)

R = flipud( Temp1');

% Define K and R such that all diagonal components of K are positive

D = diag(sign(diag(K)));
K = K * D;
R = D * R;

% Use the rotation matrix to extract the origin offset

o = -R'* K^(-1) * M(:,4);
end

