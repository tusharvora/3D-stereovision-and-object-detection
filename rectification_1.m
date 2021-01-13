%% rectification_1

% This function takes image, Rectification Transformation matrix, minimum row and colum,maximum row and column of rectified image
% Provide Rectified image as an output but not accurate 

function image_2=rectification_1(image,T,min_ro,max_co,max_ro,min_co)

% Convert the color image into gray scale image.
image = rgb2gray(image);
% Convert the data type from uint8 to double.
image = double(image);

nRow = size(image, 1);
nCol = size(image, 2);

for row2 = min_ro:max_ro
 for col2 = min_co:max_co

R_p1=T*[;xx;1];
R_p=R_p1/R_p1(3);
x=R_p(2);
y=R_p(1);
row1 = y + half_nRow;
col1 = x + half_nCol;

% Bilinear interpolation 

 if 1 <= col1 && col1 <= nCol && 1 <= row1 && row1 <= nRow
 % ..When inside the image 1,
 % ..linearly interpolate to get sub_pixel gray level.
 up = floor(row1);% convert to integer 
 down = up + 1;
 left = floor(col1);
 right = left + 1;

 if down <= nRow && right <= nCol
 intensity_1 = image(up, left);
 intensity_2 = image(down, left);
 leftIntensity = (row1 - up) * ...
 (intensity_2 - intensity_1) + intensity_1;

 intensity_1 = image(up, right);
 intensity_2 = image(down, right);
 rightIntensity = (row1 - up) * ...
 (intensity_2 - intensity_1) + intensity_1;

 intensity = (col1 - left) * ...
 (rightIntensity - leftIntensity) + leftIntensity;
 else
 intensity = image(round(row1), round(col2));
 end
 else
 % ..When ouside the image 1,
 % ..set the gray_level to be white.
 intensity = 0;
 end

 image_2(row2, col2) = intensity;
 end
end
% Display the result.
figure;
subplot(1, 2, 1);
imshow(image ./ 255);
title('Original image');
axis on;
axis image;
subplot(1, 2, 2);

imshow(image_2 ./ 255);
hold on
title('Rectified image');
axis on;
axis image;
end



