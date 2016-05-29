function [motion, structure, f] = bundleAdjustment(frame,adjustFocalLength)
%BUNDLEADJUSTMENT runs bundle adjustment on a group of cameras to adjust
% motion and structure
% Arguments:
%          frame  - a frame structure (see createFrame function for more
%               details)
%          adjustFocalLength - an optional argument that allows us to also
%               adjust the focal length in addition to the structure and motion
%
% Returns:
%          motion - 3x4xM of transformation matrices from the world frame
%               to the camera frame
%          structure - 3xN matrix of 3D points
%          f - focal length of the camera
           
f = frame.focal_length;
numCameras=size(frame.motion,3);

% convert from transformation matrix into angle axis
motion_angle_axis = zeros(3,2,numCameras);
for camera=1:numCameras
    motion_angle_axis(:,1,camera) = RotationMatrix2AngleAxis(frame.motion(:,1:3,camera));
    motion_angle_axis(:,2,camera) = frame.motion(:,4,camera);
end

structure = frame.structure;

% assume px, py=0
px = 0;
py = 0;

errors = reprojectionErrorMotStr(frame.match_idx,frame.match_points,px,py,frame.focal_length,motion_angle_axis,structure);
% fprintf('initial error = %f\n', 2*sqrt(sum(errors.^2)/length(errors)));

% bundle adjustment using lsqnonlin (Levenberg-Marquardt)
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','off');


% adjust motion and structure
[vec,resnorm,errors,exitflag] = lsqnonlin(@(x) reprojectionErrorMotStr(frame.match_idx,frame.match_points,px,py,f,x), [motion_angle_axis(:); structure(:)],[],[],options);
cut = 3*2*numCameras;
structure = reshape(vec(cut+1:end),3,[]); 
motion_angle_axis = reshape(vec(1:cut),3,2,[]);
% fprintf('error = %f\n', 2*sqrt(resnorm/length(errors)));

% if wanted, also adjust focal length
if exist('adjustFocalLength','var') && adjustFocalLength
    % adjust focal length, motion and structure
    [vec,resnorm,errors,exitflag] = lsqnonlin(@(x) reprojectionErrorMotStr(frame.match_idx,frame.match_points,px,py,x), [f; motion_angle_axis(:); structure(:)],[],[],options);
    [motion_angle_axis,structure,f] = unpackMotStrf(numCameras,vec);
%     fprintf('error = %f\n', resnorm/length(errors));
end

% revert from angle axis back to regular transformation
for camera=1:numCameras
    motion(:,:,camera) = [AngleAxis2RotationMatrix(motion_angle_axis(:,1,camera))  motion_angle_axis(:,2,camera)];    
end

