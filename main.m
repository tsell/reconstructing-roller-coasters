%% Set up.
clear all; close all; clc; rng default; warning('off','all');

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 5;

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 50;

% How many images to use when determining track width.
TRACK_WIDTH_SUBSET_SIZE = 50;

% Maximum distance from plane when fitting track planes.
PLANE_FIT_THRESHOLD = 20;

% How many example images to display.
EXAMPLE_IMAGES = 1;

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
example_image_nums = randi(numel(image_paths), 1, EXAMPLE_IMAGES);
for i=1:EXAMPLE_IMAGES
  rand_num = example_image_nums(i);
  impath = sprintf('centroid_image_%05d.png', rand_num+image_range(1)-1);
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
for i=1:EXAMPLE_IMAGES
  rand_num = example_image_nums(i);
  impath = sprintf('just_the_track_%05d.png', rand_num+image_range(1)-1);
  im = imread(image_paths{rand_num});
  [H W C] = size(im);
  pixel_list = reshape(im, H*W, C);
  centroids = knnsearch(color_centroids, double(pixel_list));
  cim = reshape(centroids, H, W);
  cim = (cim==track_color_centroid_idx);
  imwrite(cim, [0 0 0; track_color_centroid / 255], impath);
  imshow(cim, [0 0 0; track_color_centroid / 255]);
end

%% Find track width (should be ~500-1000 pixels for the orange track, not very precise).
subset_images = random_subset_images(image_paths, TRACK_WIDTH_SUBSET_SIZE);
track_width_pixels = average_track_width(subset_images, track_color_centroid_idx, color_centroids)

%% GoPros have a focal length of 14mm, roller coaster tracks are 48 inches wide.
focal_length_inches = (14 * 0.0393701);
focal_length_pixels = focal_length_inches * (track_width_pixels / 48);
K = eye(3); K(1,1) = focal_length_pixels; K(2,2) = focal_length_pixels;
cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Mini SFM, for testing.
test_image_paths = image_paths(3600:3700);
track_points = sfm(test_image_paths, track_color_centroid_idx, color_centroids, cameraParams, 1)
disp('Done computation, rendering figures...')

%% Display Figure
hold off;
figure;
axis tight auto;
pcshow(track_points,'MarkerSize',60);
saveas(gcf,'test_points.png');
v = VideoWriter('test_rotate_points.avi');
open(v);
for k = 45:135
  view(k, 45);
  writeVideo(v,getframe(gcf));
end
close(v);

%% Solve full SFM.
track_points = sfm(image_paths, track_color_centroid_idx, color_centroids, cameraParams, 0)
disp('Done computation, rendering figures...')

%% Display Figure
hold off;
figure;
axis tight auto;
pcshow(track_points);
saveas(gcf,'points.png');
v = VideoWriter('rotate_points.avi');
open(v);
for k = 45:135
  view(k, 45);
  writeVideo(v,getframe(gcf));
end
close(v);
exit;
