%% Set up.
clear all; close all; clc; rng default; warning('off','all');

% Write feature images to disk during SFM?
SAVE_FRAMES = 0;

% Which images to use?
image_folder = 'N_uV0Q2UH98'
image_range = [91 5962]

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 50

% How many images to use when determing track color.
TRACK_COLOR_SUBSET_SIZE = 50

% How many images to use when determining track width.
TRACK_WIDTH_SUBSET_SIZE = 50

% How many example images to display.
EXAMPLE_IMAGES = 2;

% How many and which frames to use in our test run of SFM.
% For example, start=91, size=3, frameskip=5 will use frames [91, 96, 101].
% Our SFM algorithm will also try the frames in between if it's unable to
% calculate the fundamental matrix using those exact frames. So you might get
% a reconstruction with frames [91, 96, 103] or [91, 99, 101] instead.
TEST_START = 1
TEST_SIZE = 100
TEST_FRAMESKIP = 20

% Get list of image paths.
image_paths = cell(diff(image_range), 1);
for i=image_range(1):image_range(2)
  image_paths{i - image_range(1) + 1} = sprintf('%s/%05d.png', image_folder, i);
end

%% Pick colors.
% Use k-means to determine the centroids of our colors.
subset_images = random_subset_images(image_paths, COLOR_SUBSET_SIZE);
disp('Clustering colors...');
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
disp('Finding track color...')
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
  figure;
  imshow(cim, [0 0 0; track_color_centroid / 255]);
end

%% Find track width (should be ~500-1000 pixels for the orange track, not very precise).
subset_images = random_subset_images(image_paths, TRACK_WIDTH_SUBSET_SIZE);
disp('Finding track width...');
track_width_pixels = average_track_width(subset_images, track_color_centroid_idx, color_centroids)

%% GoPros have a focal length of 14mm, roller coaster tracks are 48 inches wide.
focal_length_inches = (14 * 0.0393701);
focal_length_pixels = focal_length_inches * (track_width_pixels / 48);
K = eye(3); K(1,1) = focal_length_pixels; K(2,2) = focal_length_pixels;
K(3,1) = round(W/2); K(3,2) = round(H/2);
cameraParams = cameraParameters('IntrinsicMatrix', K,...
                                'RadialDistortion', [-0.000028 0]);
for i=1:EXAMPLE_IMAGES
  rand_num = example_image_nums(i);
  impath = sprintf('undistorted_%05d.png', rand_num+image_range(1)-1);
  I = undistortImage(rgb2gray(imread(image_paths{rand_num})), cameraParams);
  figure;
  imshow(I);
  saveas(gcf,impath);
end

%% Do SFM, get the camera poses.
disp('SFM...');
camera_points = sfm(...
    image_paths, track_color_centroid_idx, color_centroids, cameraParams, SAVE_FRAMES,...
    TEST_START, TEST_SIZE,  TEST_FRAMESKIP);
disp('Done computation, rendering figures...')

%% Display Figure
hold off;
figure;
axis tight auto;
plot3(camera_points(:,1), camera_points(:,2), camera_points(:,3),...
      'Color', track_color_centroid/255, 'LineStyle', '-', 'Marker', '+');
saveas(gcf,'camera_points.png');
v = VideoWriter('rotate_camera_points.avi');
open(v);
for k = 45:135
  view(k, 45);
  writeVideo(v,getframe(gcf));
end
close(v);

disp('Done!');
