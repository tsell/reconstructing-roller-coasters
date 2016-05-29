function [ new_camera_center, track_angle ] = next_center_and_angle(camera_center, image, track_color_centroid_idx, color_centroids)

% How tall each stripe should be.
STRIPE_WIDTH = 20;
% How many stripes to use to calculate the track angle.
STRIPE_COUNT = 3;

[H W C] = size(image);
image = imgaussfilt(image, 4);

% Track width of track in each stripe.
widths = zeros(STRIPE_COUNT, 1);

% Track center index of track in each stripe.
centers = zeros(STRIPE_COUNT, 1);

% Start at the bottom.
stripe_start = H-STRIPE_WIDTH;
for s=1:STRIPE_COUNT
  stripe = image(stripe_start:stripe_start+STRIPE_WIDTH,:,:);
  [Hs Ws Cs] = size(stripe);

  % As a list of pixels (one 3-column RGB per row).
  pixel_list = reshape(stripe, Hs*Ws, Cs);

  % Bucket the colors.
  colors = knnsearch(color_centroids, double(pixel_list));
  colors = reshape(colors, Hs, Ws);

  % Get the leftmost and rightmost track-colored pixel in each row.
  track_pixels_indices = colors==track_color_centroid_idx;
  lefts = zeros(STRIPE_WIDTH,1);
  rights = zeros(STRIPE_WIDTH,1);
  for r=1:STRIPE_WIDTH
    if any(track_pixels_indices(r,:))
      f = find(track_pixels_indices(r,:));
      lefts(r) = min(f);
      rights(r) = max(f);
    end
  end
  if max(rights) ~= 0
    width(s) = max(rights) - min(lefts);
    center(s) = mean([rights,lefts])
  end

end