function [ camera_points ] = sfm( image_paths, track_color_centroid_idx, color_centroids, cameraParams, ...
                                                save_images, firstframe, framecount, frameskip)

% Most of this is from http://www.mathworks.com/help/vision/examples/structure-from-motion-from-multiple-views.html

actual_frames_used = zeros(framecount, 1);

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
viewset = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
viewset = addView(viewset, viewId, 'Points', prev_points, 'Orientation', eye(3), 'Location', [0 0 0]);
disp(sprintf('Begin SFM loop (%d frames total)', framecount));
for i=1:framecount
  frameshift = 0;
  orig_frameno = firstframe + i * frameskip;
  while frameshift < frameskip
    frameno = orig_frameno + frameshift;
    disp(sprintf('Calculating with frame %d (%d+%d)', frameno, orig_frameno, frameshift));

    % Load the image.
    I = undistortImage(rgb2gray(imread(image_paths{frameno})), cameraParams);

    % Detect, extract and match features.
    curr_points = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
    curr_features = extractFeatures(I, curr_points);
    index_pairs = matchFeatures(prev_features, curr_features, 'MaxRatio', .8, 'Unique',  true);

    % Select matched points.
    matchedPoints1 = prev_points(index_pairs(:, 1));
    matchedPoints2 = curr_points(index_pairs(:, 2));

    if save_images
      figure
      showMatchedFeatures(prevI, I, matchedPoints1, matchedPoints2);
      impath = sprintf('features_%05d.png', frameno);
      saveas(gcf,impath);
      prevI = I;
    end

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    try
      [relative_orient, relative_loc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams);
      disp('Success!')
      actual_frames_used(i) = frameno;
      break
    catch ME
      warning('helperEstimateRelativePose failed with frame %d (%d+%d)', frameno, orig_frameno, frameshift);
    end

    frameshift = frameshift + 1;
    if frameshift == frameskip
      error('Used all %d inbetween frames, unable to compute fundamental matrix after frame %d',...
            frameskip, orig_frameno);
    end
  end

  % Add the current view to the view set.
  viewset = addView(viewset, i+1, 'Points', curr_points);

  % Store the point matches between the previous and the current views.
  viewset = addConnection(viewset, i, i+1, 'Matches', index_pairs(inlierIdx,:));

  % Get the table containing the previous camera pose.
  prev_pose = poses(viewset, i);
  prev_orientation = prev_pose.Orientation{1};
  prev_location = prev_pose.Location{1};

  % Compute the current camera pose in the global coordinate system
  % relative to the first view.
  orientation = prev_orientation * relative_orient;
  location = prev_location + relative_loc * prev_orientation;
  viewset = updateView(viewset, i+1, 'Orientation', orientation, 'Location', location);

  % Find point tracks across all views.
  tracks = findTracks(viewset);

  % Get the table containing camera poses for all views.
  camera_poses = poses(viewset);

  % Triangulate initial locations for the 3-D world points.
  xyzPoints = triangulateMultiview(tracks, camera_poses, cameraParams);

  % Refine the 3-D world points and camera poses.
  [xyzPoints, camera_poses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
      tracks, camera_poses, cameraParams, 'FixedViewId', 1, ...
      'PointsUndistorted', true);

  % Store the refined camera poses.
  viewset = updateView(viewset, camera_poses);

  prev_features = curr_features;
  prev_points = curr_points;
end

disp('Done!')
camera_points = cell2mat(camera_poses.Location);
