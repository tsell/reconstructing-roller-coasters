function [ track_width, new_camera_center ] = next_center(camera_center, image, track_color_centroid_idx, color_centroids, track_width_pixels)

% How tall each stripe should be.
STRIPE_HEIGHT = 20;
% How many stripes to use to calculate the track angle.
STRIPE_COUNT = 2;

[H W C] = size(image);
image = imgaussfilt(image, 4);

% Track width of track in each stripe.
widths = zeros(STRIPE_COUNT, 1);

% Track center index of track in each stripe.
centers = zeros(STRIPE_COUNT, 1);

% Start at the bottom.
stripe_start = H-STRIPE_HEIGHT;
for s=1:STRIPE_COUNT
  stripe = image(stripe_start:stripe_start+STRIPE_HEIGHT,:,:);
  stripe_start = stripe_start-STRIPE_HEIGHT-1;
  [Hs Ws Cs] = size(stripe);

  % As a list of pixels (one 3-column RGB per row).
  pixel_list = reshape(stripe, Hs*Ws, Cs);

  % Bucket the colors.
  colors = knnsearch(color_centroids, double(pixel_list));
  colors = reshape(colors, Hs, Ws);

  % Get the leftmost and rightmost track-colored pixel in each row.
  track_pixels_indices = colors==track_color_centroid_idx;
  lefts = zeros(STRIPE_HEIGHT,1);
  rights = zeros(STRIPE_HEIGHT,1);
  for r=1:STRIPE_HEIGHT
    if any(track_pixels_indices(r,:))
      f = find(track_pixels_indices(r,:));
      lefts(r) = min(f);
      rights(r) = max(f);
    end
  end
  if max(rights) ~= 0
    widths(s) = max(rights) - min(lefts);
    centers(s) = mean([rights;lefts]);
  else
    track_width = 0;
    new_camera_center = camera_center;
    return
  end
end

% TODO: fix
forward_translate = acos(widths(2)/widths(1));
horiz_translate = (centers(2) - centers(1)) / track_width_pixels;
vert_translate = track_width_pixels/widths(1) - 1;

t = [forward_translate, horiz_translate, vert_translate];
new_camera_center = camera_center + t;
track_width = widths(1) / track_width_pixels;