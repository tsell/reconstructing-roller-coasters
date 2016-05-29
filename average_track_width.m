function [ width ] = average_track_width( images, track_color_centroid_idx, color_centroids )
% How wide of a strip (in pixels) at the bottom to use for track-width determination.
BOTTOM_STRIP_WIDTH = 50;

width = 0;
no_track_images = 0;

for i=1:numel(images)
  % Get the bottom strip.
  im = images{i};
  bottom_strip = imgaussfilt(im(end-BOTTOM_STRIP_WIDTH:end,:,:), 4);
  [H W C] = size(bottom_strip);
  
  % As a list of pixels (one 3-column RGB per row).
  pixel_list = reshape(bottom_strip, H*W, C);
  
  % Bucket the colors.
  colors = knnsearch(color_centroids, double(pixel_list));
  colors = reshape(colors, H, W);

  % Get the leftmost and rightmost track-colored pixel in each row.
  track_pixels_indices = colors==track_color_centroid_idx;
  lefts = zeros(BOTTOM_STRIP_WIDTH,1);
  rights = zeros(BOTTOM_STRIP_WIDTH,1);
  for r=1:BOTTOM_STRIP_WIDTH
    if any(track_pixels_indices(r,:))
      f = find(track_pixels_indices(r,:));
      lefts(r) = min(f);
      rights(r) = max(f);
    end
  end
  if max(rights) ~= 0
    width = width + max(rights) - min(lefts);
  else
    no_track_images = no_track_images + 1;
  end
end

width = width / (numel(images) - no_track_images);
