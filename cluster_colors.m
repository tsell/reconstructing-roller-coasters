function [ color_centroids ] = cluster_colors( images )
% Use k-means to determine the centroids of our colors.
%   Input:
%     images = a cell array of RGB images.
%
%   Returns:
%     color_centroids = a NUM_COLORS x 3 matrix of RGB values.

% How many colors to distinguish (we want the track to be all one color).
NUM_COLORS = 10;
% How many pixels to sample per image.
SUBSET_SIZE = 1000 * NUM_COLORS;

% Pick a random sample of pixels from the images.
% pixels will become a SUBSET_SIZE*numel(images) x 3 array, with one column each for R, G, B.
pixels = []; 
for i=1:numel(images)
  [H W C] = size(images{i});
  im = reshape(images{i}, H*W, C); 
  new_pixels = im(randperm(H*W, SUBSET_SIZE), :); 
  pixels = [pixels; new_pixels];
end

% Run k-means on the random sample.
[idx, color_centroids] = kmeans(double(pixels), NUM_COLORS);
end
