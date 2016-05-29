function [ X ] = linearEstimate3D( x, P, image_sizes )
%LINEARESTIMATE3D Given a match in K different images, find the best 3D point
%
%   INPUTS:
%       x - measured points in each of the K images (2xK matrix)
%       P - camera matrices (K-cell of 3x4 matrices)
%       image_sizes - 2xK matrix of image sizes, 1st row are image widths, 
%           2nd row are image heights
%   OUTPUTS:
%       X - 3D point that is MLE given the K projections in different
%       images
%
% We do some preprocessing at the start of the code to ensure stability at
% runtime. You don't have to modify it.
%
% This problem is described in Multiple View Geometry, Hartley & Zisserman pg. 312

if iscell(P)
  P = cat(3,P{:});
end
K = size(P,3);

% We need to normalize x to range from -1 to 1 (recall that
% this also changes the camera projection matrices). Otherwise your
% matrices may end up being near singular.
for k = 1:K
    H = [2/image_sizes(1,k) 0 0
         0 2/image_sizes(2,k) 0
         0 0              1];
    P(:,:,k) = H*P(:,:,k);
    x(:,k) = H(1:2,1:2)*x(:,k) + H(1:2,3);
end

% IMPLEMENT THE FUNCTION BELOW
A = [];
for k = 1:K
  r1 = x(1,k)*P(3,:,k) - P(1,:,k);
  r2 = x(2,k)*P(3,:,k) - P(2,:,k);
  A = [A; r1; r2];
end

[U D V] = svd(A);
X = V(:,end);

end

