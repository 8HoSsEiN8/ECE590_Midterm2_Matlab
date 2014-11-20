clear all; close all; clc
%% Question 1:
% I had a hard time finding the answer without the focal length and pixel
% width, so I will attempt to answer the question with the following
% assumptions (unfortunately I spent majority of my time on this in class):

f = 85e-3;      % focal length is 85mm
p = 280e-6;     % pixel width is 280 um/pixel

% 3300 pixels are detected as the sphere on the camera and a 2D image of
% the sphere is a circle.  The 3300 pixels can be thought of as the area
% and from there the diameter and radius of the circle can be calculated:

Ac = 3300;      % area of the sphere in camera in pixels
disp('Radius of the sphere in camera view in pixels:')
rc = sqrt( 3300 / pi )
disp('Diameter of the sphere in camera view in pixels:')
Dc = 2 * rc

% The area of the sphere in real life with 1m Diameter would be:
Dr = 1
rr = 1 / 2
disp('Area of the sphere in meters:')
Ar = pi * rr^2

% Given the fact that each pixel is 280 um and that the ratio of
% real/camera diameter should be equal to the ratio of the distance/focal
% length then one can calculate the distance as:
disp('Calculated distance to the center of the sphere in meters:')
d = f * ( Dr / (Dc * p) )

% However, the question is asking about "closest" point to the sphere which
% would be 1 radius closer than the calculated distance above:
disp('Closest distance to the sphere in meters:')
real_d = d - rr

%% Question 2:
% With the given focal length, baseline and pixel size the disparity and
% distance can be calculated:

f = 85e-3;      % focal length is 85mm
p = 280e-6;     % pixel width is 280 um/pixel
b = 0.4;        % baseline in m

disp('Object pixel coordinate in right camera:')
Xr = [138, 73]
disp('Object pixel coordinate in left camera:')
Xl = [160, 71]

disp('Disparity of object detected in two cameras:')
d = int16(pdist2(Xr, Xl,'euclidean'))

disp('Depth or distance in meters is calculated as:')
dist = (f * b) / (d * p)



