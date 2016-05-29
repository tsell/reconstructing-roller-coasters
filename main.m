%% Set up.
clear all; close all; clc; rng default;

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 5;
% How many colors to distinguish (we want the track to be all one color).
NUM_COLORS = 10;

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 100;

% Which images to use?
image_folder = 'N_uV0Q2UH98';
image_range = [91 6119];

% Get list of image paths.
image_paths = cell(diff(image_range), 1);
for i=image_range(1):image_range(2)
  image_paths{i - image_range(1) + 1} = sprintf('%s/%05d.png', image_folder, i);
end

%% Pick colors.
% Select a random subset of images.
subset_paths = datasample(image_paths, COLOR_SUBSET_SIZE);
subset_images = cell(COLOR_SUBSET_SIZE, 1);
for i=1:COLOR_SUBSET_SIZE
  subset_images{i} = imread(subset_paths{i});
end

% Use k-means to determine the centroids of our colors.
color_centroids = cluster_colors(subset_images, NUM_COLORS)

%% Find track color.
% Select a random subset of images.
subset_paths = datasample(image_paths, TRACK_COLOR_SUBSET_SIZE);
subset_images = cell(TRACK_COLOR_SUBSET_SIZE, 1);
for i=1:TRACK_COLOR_SUBSET_SIZE
  subset_images{i} = imread(subset_paths{i});
end

halfedge_histogram = track_color(subset_images, color_centroids)
