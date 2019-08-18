function [] = visualizeKeypoints2D(img, keyp)
% VISUALIZEWIREFRAME2D  Takes in a 2D car wireframe (2 x 14 matrix), and
% plots it in 2D (on a given image) while appropriately connecting vertices


% Declare global variables
%globals;

% Number of keypoints for the car class
numKps = size(keyp,2);

% Generate distinguishable colors with respect to a white background
colors = distinguishable_colors(numKps, [0, 0, 0]);

% Display the image
imshow(img);
% Hold on, to plot the wireframe
hold on;

% Create a scatter plot of the wireframe vertices
scatter(keyp(1,:), keyp(2,:), repmat(20, 1, numKps), colors, 'filled');