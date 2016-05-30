function [ track_points ] = sfm( image_paths, color_centroids, track_color_centroid_idx)
% Initialize with the first frame.
colorimage = imread(image_paths{1});
% Manual tweaking: crop out the six flags logo.
colorimage = colorimage(:,1:1600,:);
image2 = rgb2gray(colorimage);
points = detectSURFFeatures(image2);
[features2,valid_points2] = extractFeatures(image2,points);

% Show off our features.
figure
imshow(image2);
hold on
plot(points.selectStrongest(1000));
saveas(gcf,'features.png');

% Collect track points.
track_points = [];

% Rotation and translation are cumulative, remember.
cR = eye(3);
cT = [0,0,0];
camMatrix2 = cameraMatrix(cameraParams, cR, -cT*cR');

disp('Begin SFM loop')
for i=2:numel(image_paths)
  % Figure out the image's colors.
  colorimage = imread(image_paths{i});
  % Manual tweaking: crop out the six flags logo.
  colorimage = colorimage(:,1:1600,:);
  [H W C] = size(colorimage);
  pixel_list = reshape(colorimage, H*W, C);

  % Replace image1 with image2, then get a new image2.
  image1 = image2;
  image2 = rgb2gray(colorimage);
  features1 = features2;
  valid_points1 = valid_points2;

  % Detect features.
  points = detectSURFFeatures(image2);
  [features2,valid_points2] = extractFeatures(image2,points);
  
  % Find correspondences.
  index_pairs = matchFeatures(features1, features2);
  matchedPoints1 = valid_points1(index_pairs(:,1),:);
  matchedPoints2 = valid_points2(index_pairs(:,2),:);

  % Estimate the fundamental matrix F.
  [F, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','RANSAC','NumTrials',2000);

  % Solve SFM between our two frames.
  inliers1 = matchedPoints1(epipolarInliers,:);
  inliers2 = matchedPoints2(epipolarInliers,:);
  [R, t] = cameraPose(F, cameraParams, inliers1, inliers2);

  % Rotations and translations are cumulative (only frame 1 should be at the origin).
  camMatrix1 = camMatrix2;
  cR = R * cR;
  cT = t + cT;
  camMatrix2 = cameraMatrix(cameraParams, cR', -cT*cR');

  % Compute the 3-D points.
  points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

  % Compute the corresponding colors of each 3D points.
  colorIdx = sub2ind([H,W], round(matchedPoints2.Location(:, 2)), round(matchedPoints2.Location(:, 1)));
  pointColors = pixel_list(colorIdx, :);
  pc = pointCloud(points3D, 'Color', pointColors);
  
  % Keep just the points which match the track color.
  point_color_centroids = knnsearch(color_centroids, double(pointColors));
  track_point_idxs = find(point_color_centroids == track_color_centroid_idx);
  pc = select(pc, track_point_idxs);
  if isempty(track_points)
    track_points = pc;
  else
    track_points = pcmerge(track_points, pc, 10);
  end

  % We don't need a million warnings about singular matrices.
  [a, MSGID] = lastwarn(); warning('off', MSGID)
end
end
