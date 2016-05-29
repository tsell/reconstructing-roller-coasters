function frame=removeOutliers(frame,thresh)
%REMOVEOUTLIERS given a frame, removes the outlier points 
% Arguments:
%          frame  - a frame structure (see createFrame function for more
%               details)
%          thresh - an optional argument that allows us to also
%               set a threshold (in pixels) of how far the reprojection
%               error can be for each point
%
% Returns:
%          frame - same frame as before, but with structure and match
%          fields potentially changed
if exist('thresh','var')
    thresh = thresh^2;
else
    thresh = 10^2; % square it so that we don't need to square root everytime
end

threshold_in_degree = 2;
threshold_in_cos = cos(threshold_in_degree / 180 * pi);

for c=1:size(frame.match_idx,1)
    X = frame.K * transformPtsByRt(frame.structure,frame.motion(:,:,c));
    xy = X(1:2,:) ./ X([3 3],:);
    selector = find(frame.match_idx(c,:)~=0);
    
    diff = xy(:,selector) - frame.match_points(:,frame.match_idx(c,selector));
    
    outliers = sum(diff.^2,1) > thresh;
    
%     if sum(outliers)>0
%         fprintf('remove %d outliers outof %d points with reprojection error bigger than %f pixels\n',sum(outliers),length(outliers), sqrt(thresh));
%     end

    pts2keep = true(1,size(frame.structure,2));
    
    pts2keep(selector(outliers)) = false;
    
    frame.structure = frame.structure(:,pts2keep);

    frame.match_idx = frame.match_idx(:,pts2keep);
    
end

% return

% Check viewing angle

num_frames = size(frame.motion,3);
positions = zeros(3, num_frames);

for ii = 1:num_frames
  Rt = frame.motion(:, :, ii);
  positions(:, ii) = - Rt(1:3, 1:3)' * Rt(:, 4);
end

view_dirs = zeros(3, size(frame.structure, 2), num_frames);

for c = 1:size(frame.match_idx, 1)
  selector = find(frame.match_idx(c,:)~=0);
  camera_view_dirs = bsxfun(@minus, frame.structure(:, selector), positions(:, c));
  dir_length = sqrt(sum(camera_view_dirs .* camera_view_dirs));
  camera_view_dirs = bsxfun(@times, camera_view_dirs, 1 ./ dir_length);
  view_dirs(:, selector, c) = camera_view_dirs;
end

for c1 = 1:size(frame.match_idx, 1)
  for c2 = 1:size(frame.match_idx, 1)
    if c1 == c2
      continue
    end
    selector = find(frame.match_idx(c1,:)~=0 & frame.match_idx(c2,:)~= 0);
    view_dirs_1 = view_dirs(:, selector, c1);
    view_dirs_2 = view_dirs(:, selector, c2);
    cos_angles = sum(view_dirs_1 .* view_dirs_2);
    outliers = cos_angles > threshold_in_cos;
    
%     if sum(outliers)>0
%         fprintf('remove %d outliers outof %d points with view angle less than %f degrees\n',sum(outliers),length(outliers), threshold_in_degree);
%     end

    pts2keep = true(1,size(frame.structure,2));
    pts2keep(selector(outliers)) = false;
    frame.structure = frame.structure(:,pts2keep);
    frame.match_idx = frame.match_idx(:,pts2keep);
  end
end
end

function Y3D = transformPtsByRt(X3D, Rt, isInverse)

if nargin<3 || ~isInverse
    Y3D = Rt(:,1:3) * X3D + repmat(Rt(:,4),1,size(X3D,2));
else
    Y3D = Rt(:,1:3)' * (X3D - repmat(Rt(:,4),1,size(X3D,2)));
end
end
