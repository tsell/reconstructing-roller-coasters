%% Set up.
clear all; close all; clc; rng default;
addpath('sfm');

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 5;

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 20;

% How many images to use when determining track width.
TRACK_WIDTH_SUBSET_SIZE = 30;

% Which images to use?
image_folder = 'N_uV0Q2UH98';
image_range = [91 6119];

% Get list of image paths.
image_paths = cell(diff(image_range), 1);
for i=image_range(1):image_range(2)
  image_paths{i - image_range(1) + 1} = sprintf('%s/%05d.png', image_folder, i);
end

%% Pick colors.
% Use k-means to determine the centroids of our colors.
subset_images = random_subset_images(image_paths, COLOR_SUBSET_SIZE);
color_centroids = cluster_colors(subset_images)

%% Find track color. (should be ~(200,125,70) for the orange track).
subset_images = random_subset_images(image_paths, TRACK_COLOR_SUBSET_SIZE);
[track_color_centroid, track_color_centroid_idx] = track_color(subset_images, color_centroids)

%% Find track width (should be either ~870 or ~280 pixels for the orange track).
subset_images = random_subset_images(image_paths, TRACK_WIDTH_SUBSET_SIZE);
track_width_pixels = average_track_width(subset_images, track_color_centroid_idx, color_centroids)

%% We're going to use track_width_pixels as scale factor, because we arbitrarily declare the real track width 1
K = eye(3); K(1,1) = track_width_pixels; K(2,2) = track_width_pixels;

%% Solve for [R t] matrices between frames.

% TESTING ONLY: use a small set of frames.
image_range = [3000 3010]
image_paths = cell(diff(image_range), 1);
for i=image_range(1):image_range(2)
  image_paths{i - image_range(1) + 1} = sprintf('%s/%05d.png', image_folder, i);
end

% Initialize features in last image, which should be similar to the first
% image because roller coasters are a loop.
image2 = rgb2gray(imread(image_paths{end}));
points = detectSURFFeatures(image2);
[features2,valid_points2] = extractFeatures(image2,points);
track_points = zeros(numel(image_paths) + 1, 3);
for i=1:numel(image_paths)
  % Replace image1 with image2, then get a new image2.
  image1 = image2;
  image2 = rgb2gray(imread(image_paths{i}));
  [H W] = size(image2);
  features1 = features2;
  valid_points1 = valid_points2;

  % Detect features.
  points = detectSURFFeatures(image2);
  [features2,valid_points2] = extractFeatures(image2,points);
  
  % Find correspondences.
  index_pairs = matchFeatures(features1, features2);
  matchedPoints1 = valid_points1(index_pairs(:,1),:);
  matchedPoints2 = valid_points2(index_pairs(:,2),:);

  % Estimate the fundamental matrix F.
  F = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','RANSAC','NumTrials',2000,'DistanceThreshold',1e-4);
  E = K' * F * K;

  % Solve SFM.
  Rt = computeRTFromE(E, [matchedPoints1.Location'; matchedPoints2.Location'], K, H, W);

  track_points(i+1,:) = Rt * [track_points(i,:)'; 1];
end
figure();
plot3(track_points(:,1), track_points(:,2), track_points(:,3), '.');
