%% Set up.
clear all; close all; clc; rng default;

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 5;

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 20;

% How many images to use when determining track width.
TRACK_WIDTH_SUBSET_SIZE = 30;

% Which images to use?
image_folder = 'N_uV0Q2UH98';
image_range = [91 6119];
image_range = [3000 3100];

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

%% Specialized SFM based on angle of track.
width = 1;
camera_center = [0 0 0];
track_norm = [0 0 0];
camera_centers = zeros(numel(image_paths)+1, 3);
track_norms = zeros(numel(image_paths)+1, 3);
for i=1:numel(image_paths)
  image = imread(image_paths{i});
  [width camera_center track_angle] = next_center_and_angle(camera_center, image, track_color_centroid_idx, color_centroids, track_width_pixels);
  widths(i+1,:) = width;
  camera_centers(i+1,:) = camera_center;
  track_norms(i+1,:) = track_norm;
end
figure();
plot3(camera_centers(:,1), camera_centers(:,2), camera_centers(:,3), '.');
