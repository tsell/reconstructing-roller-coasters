function [ track_points ] = sfm( image_paths, track_color_centroid_idx, color_centroids, cameraParams, save_images)

%% SFM Step One
% Find the camera poses for each frame.
disp('First SFM step.')
% Initialize with the first frame.
I = undistortImage(rgb2gray(imread(image_paths{1})), cameraParams);
prevI = I;

% Crop the images to remove logos and edge distortions.
border = 100;
roi = [border, border,...
       size(I, 2) - 2*border, size(I, 1) - 2*border];
prev_points = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Show off our features.
if save_images
  figure
  imshow(I);
  hold on
  plot(prev_points);
  saveas(gcf,'features.png');
end

prev_features = extractFeatures(I, prev_points);
% Create an empty viewSet object to manage the data associated with each view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prev_points, 'Orientation', eye(3), 'Location', [0 0 0]);
disp('Begin SFM loop')
for i=2:numel(image_paths)
  disp(sprintf('%d/%d', i, numel(image_paths)))
  % Load the image.
  I = undistortImage(rgb2gray(imread(image_paths{i})), cameraParams);

  % Detect, extract and match features.
  curr_points = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
  curr_features = extractFeatures(I, curr_points);
  index_pairs = matchFeatures(prev_features, curr_features, 'MaxRatio', .9, 'Unique',  true);

  % Select matched points.
  matchedPoints1 = prev_points(index_pairs(:, 1));
  matchedPoints2 = curr_points(index_pairs(:, 2));

  if save_images
    figure
    showMatchedFeatures(prevI, I, matchedPoints1, matchedPoints2);
    impath = sprintf('features_%05d.png', i);
    saveas(gcf,impath);
    prevI = I;
  end

  % Estimate the camera pose of current view relative to the previous view.
  % The pose is computed up to scale, meaning that the distance between
  % the cameras in the previous view and the current view is set to 1.
  % This will be corrected by the bundle adjustment.
  [relative_orient, relative_loc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams);

  % Add the current view to the view set.
  vSet = addView(vSet, i, 'Points', curr_points);

  % Store the point matches between the previous and the current views.
  vSet = addConnection(vSet, i-1, i, 'Matches', index_pairs(inlierIdx,:));

  % Get the table containing the previous camera pose.
  prev_pose = poses(vSet, i-1);
  prev_orientation = prev_pose.Orientation{1};
  prev_location = prev_pose.Location{1};

  % Compute the current camera pose in the global coordinate system
  % relative to the first view.
  orientation = prev_orientation * relative_orient;
  location = prev_location + relative_loc * prev_orientation;
  vSet = updateView(vSet, i, 'Orientation', orientation, 'Location', location);

  % Find point tracks across all views.
  tracks = findTracks(vSet);

  % Get the table containing camera poses for all views.
  camPoses = poses(vSet);

  % Triangulate initial locations for the 3-D world points.
  xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

  % Refine the 3-D world points and camera poses.
  [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
      tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
      'PointsUndistorted', true);

  % Store the refined camera poses.
  vSet = updateView(vSet, camPoses);

  prev_features = curr_features;
  prev_points = curr_points;
end

camPoses = poses(vSet);
track_points = cell2mat(camPoses.Location);
