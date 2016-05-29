function [ color_centroids ] = cluster_colors( images, num_colors )
% Use k-means to determine the centroids of our colors.
%   Input:
%     images = a cell array of RGB images.
%     num_colors = number of clusters to use in k-means
%
%   Returns:
%     color_centroids = a num_colors x 3 matrix of RGB values.

sprintf('cluster_colors: Images is of size %d \n', numel(images));
sprintf('cluster_colors: Number of colors: %d', num_colors);

SUBSET_SIZE = 100 * num_colors;

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
[idx, color_centroids] = kmeans(double(pixels), num_colors);
end
