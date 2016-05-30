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

% Output image of centroid-alized pixels.
for i=1:10
  rand_num = randi(numel(image_paths)); 
  impath = sprintf('centroid_image_%05d.png', rand_num+image_range(1)-1)
  im = imread(image_paths{rand_num});
  [H W C] = size(im);
  pixel_list = reshape(im, H*W, C);
  centroids = knnsearch(color_centroids, double(pixel_list));
  cim = reshape(centroids, H, W);
  imwrite(cim, color_centroids / 255, impath);
  imshow(cim, color_centroids / 255);
end

%% Find track color. (should be ~(200,125,70) for the orange track).
subset_images = random_subset_images(image_paths, TRACK_COLOR_SUBSET_SIZE);
[track_color_centroid, track_color_centroid_idx] = track_color(subset_images, color_centroids)

% Output image of just track-colored pixels.
for i=1:10
  rand_num = randi(numel(image_paths)); 
  impath = sprintf('centroid_image_%05d.png', rand_num+image_range(1)-1)
  impath = sprintf('just_the_track_%05d.png', rand_num+image_range(1)-1)
  im = imread(image_paths{rand_num});
  [H W C] = size(im);
  pixel_list = reshape(im, H*W, C);
  centroids = knnsearch(color_centroids, double(pixel_list));
  cim = reshape(centroids, H, W);
  cim = (cim==track_color_centroid_idx);
  imwrite(cim, [0 0 0; track_color_centroid / 255], impath);
  imshow(cim, [0 0 0; track_color_centroid / 255]);
end

%% Find track width (should be either ~870 or ~280 pixels for the orange track).
subset_images = random_subset_images(image_paths, TRACK_WIDTH_SUBSET_SIZE);
track_width_pixels = average_track_width(subset_images, track_color_centroid_idx, color_centroids)

%% GoPros have a focal length of 14mm, roller coaster tracks are 48 inches wide.
focal_length_inches = (14 * 0.0393701);
focal_length_pixels = focal_length_inches * (track_width_pixels / 48);
K = eye(3); K(1,1) = focal_length_pixels; K(2,2) = focal_length_pixels;
cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Solve SFM.

% TESTING ONLY: use a small set of frames.
image_range = [91 6119];
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
track_point_colors = [];

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

  % Solve SFM between our two frames.
  inliers1 = matchedPoints1(epipolarInliers,:);
  inliers2 = matchedPoints2(epipolarInliers,:);
  [R, t] = cameraPose(F, cameraParams, inliers1, inliers2);

  % Rotations and translations are cumulative (only frame 1 should be at the origin).
  camMatrix1 = cameraMatrix(cameraParams, cR', -cT*cR');
  cR = R * cR;
  cT = cT * R' + t;
  camMatrix2 = cameraMatrix(cameraParams, cR', -cT*cR');

  % Compute the 3-D points.
  points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

  % Compute the corresponding colors of each 3D points.
  colorIdx = sub2ind([H,W], round(matchedPoints2.Location(:, 2)), round(matchedPoints2.Location(:, 1)));
  pointColors = pixel_list(colorIdx, :);
  pc = pointCloud(points3D, 'Color', pointColors);
  
  % Keep just the points which match the track color.
  point_color_centroids = knnsearch(color_centroids, double(pointColors));
  track_point_idxs = find(point_color_centroids == track_color_centroid_idx);
  pc = select(pc, track_point_idxs);
  if isempty(track_points)
    track_points = pc;
  else
    track_points = pcmerge(track_points, pc, 10);
  end
end

%% Display Figure
figure;
axis tight auto;
pcshow(track_points);
v = VideoWriter('rotate_points.avi');
open(v);
for k = 45:405
  view(k, 45);
  writeVideo(v,getframe(gcf));
end
close(v);
