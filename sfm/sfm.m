%% Setup
clear all; close all; clc;
addpath(genpath('functions'));
% need to set this random seed to produce exact same result
s = RandStream('mcg16807','Seed',1); 
RandStream.setGlobalStream(s);

% Load the images and set known data
images{1}='images/B21.jpg';
images{2}='images/B22.jpg';
images{3}='images/B23.jpg';
images{4}='images/B24.jpg';
images{5}='images/B25.jpg';
load('data.mat');
focal_length = 719.5459;
%% PART A TEST (CTRL+ENTER TO RUN THIS PART ONLY)
fprintf('------------PART A------------\n');
% Get the matches between the first pair of cameras
matches = all_matches{1};

% Get image size
I = imread(images{1}); [im_height, im_width] = size(I);

% Construct Essential Matrix
K = eye(3); K(1,1) = focal_length; K(2,2) = focal_length;
E = K'*F{1}*K;

% Print out answer (should be four 3x4 matrices) after doing only part A
% compare to example_Rt - at least one of the matrices should be close.
fprintf('Check your matrices against the example R,T.\n');
computeRTFromE(E, matches, K, im_width, im_height) 
example_Rt = [0.9736   -0.0988   -0.2056    0.9994;
              0.1019    0.9948    0.0045   -0.0089;
              0.2041   -0.0254    0.9786    0.0331]
%% PART B TEST (CTRL+ENTER TO RUN THIS PART ONLY)
fprintf('------------PART B------------\n');
P{1} = K * [eye(3) [0;0;0]];
P{2} = K * example_Rt;
m = reshape(matches(:,1),2,2);
pt = linearEstimate3D(m,P,repmat([im_height;im_width],1,2));
pt = pt / pt(4);

% Check against this point, difference should be negligible
example_pt = [0.6774;  -1.1029;   4.6621;   1];
fprintf('Difference should be close to zero.\n');
difference = example_pt - pt

%% PART C TEST (CTRL+ENTER TO RUN THIS PART ONLY)
fprintf('------------PART C------------\n');
m = reshape(matches(:,1),2,2);
[error, Jacobian] = reprojectionError(pt(1:3), m, P);

% Check error/Jacobian against this (difference should be zero)
example_error = [-0.0151;   -0.5116;    0.0002;    0.5062];
example_J = [154.3393     0      -22.4242;
                0      154.3393   36.5104;
             141.8799  -14.2774  -56.2023;
              21.9791  149.5064   32.2333];
fprintf('Difference should be close to zero.\n');
e_difference = error - example_error          
e_difference = Jacobian - example_J


%% PART D TEST (CTRL+ENTER TO RUN THIS PART ONLY)
fprintf('------------PART D------------\n');
lin_pt = linearEstimate3D(testx,testP,repmat([im_height;im_width],1,4));
lin_pt = lin_pt / lin_pt(4)

nonlin_pt = nonlinEstimate3D(testx,testP,repmat([im_height;im_width],1,4));
nonlin_pt = nonlin_pt / nonlin_pt(4)

[lin_error, Jacobian] = reprojectionError(lin_pt(1:3), testx, testP);
[nonlin_error, Jacobian] = reprojectionError(nonlin_pt(1:3), testx, testP);


% Check to make sure reprojection error went down
fprintf('Reprojection error should have went down.\n');
lin_error = norm(lin_error)
nonlin_error = norm(nonlin_error)

%% PART E: Runs the entire Structure From Motion Pipeline
fprintf('------------PART E------------\n');
% For each sequential pair of images, triangulate/reconstruct the 3D points from that pair
for i = 1:length(images)-1
    % read in images and conversion to single is recommended 
    I = imread(images{i});   J = imread(images{i+1});
    I = single(rgb2gray(I)); J = single(rgb2gray(J));

    % Create the frame between each sequential pair
    % Read the frame documentation in the createFrame() function
    frames{i} = createFrame(all_matches{i}, focal_length, F{i}, size(I));
    
    % Run bundle adjustment to get updated values for bundle adjustment
    [frames{i}.motion, frames{i}.structure, frames{i}.focal_length] = bundleAdjustment(frames{i});
end
% Merge all pairs' motion and structure
mergedFrame = mergeAllFrames(frames);
figure();
plot3(mergedFrame.structure(1,:), mergedFrame.structure(2,:), mergedFrame.structure(3,:), '.');
axis([-5 5 -10 5 0 15]);
