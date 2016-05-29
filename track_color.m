function [ track_color_centroid ] = track_color(images, color_centroids)

% How wide of a strip (in pixels) at the bottom to use for track-color determination.
BOTTOM_STRIP_WIDTH = 10;
% How many segments to break the bottom strip into for track-color determination.
BOTTOM_STRIP_SEGMENTS = 12;

[num_colors x] = size(color_centroids);

% Initialize the histogram for our half-edge colors.
halfedge_histogram = zeros(BOTTOM_STRIP_SEGMENTS, num_colors);

% Build the histogram using our random subset of images.
for i=1:numel(images)
  im = images{i};
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

    % Figure out which color each halfedge pixel is closest to.
    colors = knnsearch(color_centroids, double(block_pixel_list));

    % Update histogram for this block.
    [h, e] = histcounts(colors, num_colors);
    halfedge_histogram(blocknum,:) = halfedge_histogram(blocknum,:) + h;

    % Next block.
    blockleft = blockright + 1;
    blockright = blockleft+blockwidth-1;
    blocknum = blocknum + 1;
  end
end

halfedge_histogram

% Find the color with the most zeroes in its column.
% This will be the track color, because the track ONLY appears in the center group of
% sections. All other colors should appear at least once somewhere.
[m track_color_idx] = max(sum(halfedge_histogram==0, 1));
track_color_centroid = color_centroids(track_color_idx, :);

