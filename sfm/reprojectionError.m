function [ error, J ] = reprojectionError( point3D, image_points, projection_matrices )
%REPROJECTIONERROR Computes reprojection error, given 3d point and its
% matching points
%   INPUTS:
%       point3D - The 3x1 point in 3D space that provides the
%           correspondences in each of the cameras
%       image_points - The 2xK  coordinates in each of the K images
%       projection_matrices - a list of 3x4xK P matrices between the K cameras  
%   OUTPUTS:
%       error - The 2Kx1 reprojection error
%       J - The 2Kx3 jacobian matrix

    % reformatting data in case projection_matrices is a cell array of matrices
    if iscell(projection_matrices)
      projection_matrices = cat(3,projection_matrices{:});
    end
    
    % TODO: Implement this function
    X = [point3D', 1]';

    error = [];
    J = [];
    for k = 1:length(image_points)
      P = projection_matrices(:,:,k);
      y = P * X;
      x = [y(1); y(2)] * (1/y(3));
      error = [error; x - image_points(:,k)];
      grade_x11 = (y(3)*P(1,1)-P(3,1)*y(1)) / (y(3)^2);
      grade_x12 = (y(3)*P(2,1)-P(3,1)*y(2)) / (y(3)^2);
      grade_x21 = (y(3)*P(1,2)-P(3,2)*y(1)) / (y(3)^2);
      grade_x22 = (y(3)*P(2,2)-P(3,2)*y(2)) / (y(3)^2);
      grade_x31 = (y(3)*P(1,3)-P(3,3)*y(1)) / (y(3)^2);
      grade_x32 = (y(3)*P(2,3)-P(3,3)*y(2)) / (y(3)^2);
      J = [J; grade_x11 grade_x21 grade_x31; grade_x12 grade_x22 grade_x32];
    end
end

