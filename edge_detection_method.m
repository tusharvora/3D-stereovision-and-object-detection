%% Edge detection using in built function 

close all;
A=imread('image1','jpg'); % reading image

Bsum=rgb2gray(A); % converting rgb to gray scale
figure(),imshow(Bsum);

a=edge(Bsum,'roberts'); % Roberts edge detection method
b=edge(Bsum,'sobel');   % Sobel edge detection method
c=edge(Bsum,'prewitt'); % Prewitt edge detection method
d=edge(Bsum,'canny');   % Canny edge detection method
e=edge(Bsum,'log');     % Log edge detection method
 
figure(3);
subplot(2,3,1),imshow(A),title('Original Image');
subplot(2,3,2),imshow(a),title('roberts');
subplot(2,3,3),imshow(b),title('sobel');
subplot(2,3,4),imshow(c),title('prewitt');
subplot(2,3,5),imshow(d),title('canny Image');
subplot(2,3,6),imshow(e),title('log Image');



