function [ subset_images ] = random_subset_images( image_paths, subset_size )
% Select a random subset of images and load them into memory.
subset_paths = datasample(image_paths, subset_size);
subset_images = cell(subset_size, 1);
for i=1:subset_size
  subset_images{i} = imread(subset_paths{i});
end
