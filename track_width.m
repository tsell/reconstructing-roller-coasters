function [ width ] = average_track_width( images, track_color_centroid_idx, color_centroids )
% How wide of a strip (in pixels) at the bottom to use for track-width determination.
BOTTOM_STRIP_WIDTH = 10;

width = 0;

for i=1:numel(images)
  % Get the bottom strip.
  im = imgaussfilt(images{i}, 2);
  bottom_strip = im(end-BOTTOM_STRIP_WIDTH:end,:,:);
  [H W C] = size(bottom_strip);
  
  % As a list of pixels (one 3-column RGB per row).
  pixel_list = reshape(bottom_strip, H*W, C);
  
  % Bucket the colors.
  colors = knnsearch(color_centroids, double(pixel_list));
  colors = reshape(colors, H, W);

  % Get the leftmost and rightmost track-colored pixel in each row.
  track_pixels_indices = colors==track_color_centroid_idx;
  for r=1:BOTTOM_STRIP_WIDTH
    if any(track_pixels_indices(r,:))
      width = width + range(find(track_pixels_indices(r,:)));
    end
  end
end

width = width / (BOTTOM_STRIP_WIDTH * numel(images));
