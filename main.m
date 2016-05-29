%% Set up.
clear all; close all; clc; rng default;

% How many images to use when determining colors.
COLOR_SUBSET_SIZE = 10;
% How many colors to distinguish (we want the track to be all one color).
NUM_COLORS = 10;
% How wide of a strip (in pixels) at the bottom to use for track-color determination.
BOTTOM_STRIP_WIDTH = 20;
% How many segments to break the bottom strip into for track-color determination.
BOTTOM_STRIP_SEGMENTS = 8;

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
subset_paths = datasample(image_paths, COLOR_SUBSET_SIZE)
subset_images = cell(COLOR_SUBSET_SIZE, 1);
for i=1:COLOR_SUBSET_SIZE
  subset_images{i} = imread(subset_paths{i});
end

% Use k-means to determine the centroids of our colors.
color_centroids = cluster_colors(subset_images, NUM_COLORS)

% Initialize the histogram for our half-edge colors.
halfedge_histogram = zeros(BOTTOM_STRIP_SEGMENTS, NUM_COLORS);

% Build the histogram using our random subset of images.
for i=1:COLOR_SUBSET_SIZE
  im = subset_images{i};
  bottom_strip = im(end-BOTTOM_STRIP_WIDTH:end,:,:);
  [H W C] = size(bottom_strip);
  % Detect edges. We use the hue value to avoid problems
  % where two different colors with the same saturation look the same in grayscale.
  % This is good enough for our purposes, we don't need a (slower, more complex)
  % real RGB edge detection algorithm.
  hsvim = rgb2hsv(im);
  ed = edge(hsvim(end-BOTTOM_STRIP_WIDTH:end,:,1));

  % We don't want the actual edge, we want the pixels on either side.
  halfedges = imtranslate(ed, [1, 0]) | imtranslate(ed, [-1, 0]);
  
  % Match the colors in those edges to the centroids we determined earlier.
  blockwidth = floor(W/BOTTOM_STRIP_SEGMENTS);
  blockleft = 1;
  blockright = blockleft+blockwidth-1;
  blocknum = 1;
  while blockright <= W
    % Get list of pixels in this block as 3 columns (one row of RGB per pixel)
    imblock = bottom_strip(:, blockleft:blockright, :);
    imblock_pixel_list = reshape(imblock, H*blockwidth, C);

    % Get a mask of which pixels are halfedges.
    edblock = halfedges(:, blockleft:blockright);
    edblock_pixel_list = reshape(edblock, H*blockwidth, 1);

    % Do the masking to get a list of pixels that are halfedges.
    block_pixel_list = imblock_pixel_list(find(edblock_pixel_list), :);

    % Figure out which color each halfedge pixel is.
    colors = knnsearch(color_centroids, double(block_pixel_list));

    % Update histogram for this block.
    [h, e] = histcounts(colors, NUM_COLORS);
    halfedge_histogram(blocknum,:) = halfedge_histogram(blocknum,:) + h;

    % Next block.
    blockleft = blockright + 1;
    blockright = blockleft+blockwidth-1;
    blocknum = blocknum + 1;
  end
end

halfedge_histogram
