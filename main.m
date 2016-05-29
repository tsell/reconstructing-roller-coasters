%% Set up.
clear all; close all; clc; rng default;

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 5;

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 20;

% How many images to use when determining track width.
TRACK_WIDTH_SUBSET_SIZE = 30;

% Maximum distance from plane when fitting track planes.
PLANE_FIT_THRESHOLD = 20;

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
K = eye(3); K(3,3) = track_width_pixels;
cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Solve for [R t] matrices between frames.

% TESTING ONLY: use a small set of frames.
image_range = [91 6119];
image_range = [91 100]
image_paths = cell(diff(image_range), 1);
for i=image_range(1):image_range(2)
  image_paths{i - image_range(1) + 1} = sprintf('%s/%05d.png', image_folder, i);
end

colorimage = imread(image_paths{1});
image2 = rgb2gray(colorimage);
points = detectSURFFeatures(image2);
[features2,valid_points2] = extractFeatures(image2,points);

% Collect track points.
track_points = [];

% Rotation and translation are cumulative, remember.
cR = eye(3);
cT = [0,0,0];

for i=2:numel(image_paths)
  % Figure out the image's colors.
  colorimage = imread(image_paths{i});
  [H W C] = size(colorimage);
  pixel_list = reshape(colorimage, H*W, C);

  % Replace image1 with image2, then get a new image2.
  image1 = image2;
  image2 = rgb2gray(colorimage);
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
  [F, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','MSAC','NumTrials',2000);

  % Solve SFM.
  inliers1 = matchedPoints1(epipolarInliers,:);
  inliers2 = matchedPoints2(epipolarInliers,:);
  [R, t] = cameraPose(F, cameraParams, inliers1, inliers2);
  camMatrix1 = cameraMatrix(cameraParams, cR', -cT*cR');
  cR = R * cR;
  cT = cT * R' + t;
  camMatrix2 = cameraMatrix(cameraParams, cR', -cT*cR');

  % Compute the 3-D points.
  points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

  % Compute the corresponding colors of each 3D points.
  colorIdx = sub2ind([H,W], round(matchedPoints2.Location(:, 2)), round(matchedPoints2.Location(:, 1)));
  pointColors = pixel_list(colorIdx, :);
  
  % Keep just the points which match the track color.
  point_color_centroids = knnsearch(color_centroids, double(pointColors));
  track_point_idxs = find(point_color_centroids == track_color_centroid_idx);
  if isempty(track_points)
    track_points = select(pointCloud(points3D), track_point_idxs);
  else
    track_points = pcmerge(track_points, select(pointCloud(points3D), track_point_idxs), 1);
  end
end

%% Display Figure
figure;
axis tight auto;
pcshow(track_points);
v = VideoWriter('rotate_points.avi');
open(v);
for k = 1:180
  view(k, 45);
  writeVideo(v,getframe(gcf));
end
close(v);
