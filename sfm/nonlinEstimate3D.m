function [ X ] = nonlinEstimate3D( x, P, image_sizes )
%NONLINESTIMATE3D %Given a match in K different images, find the best 3D point
%   As outlined in Zisserman pg. 95-99, 314 - 315 
%   INPUTS:
%       x - measured points in each of the K images (2xK matrix)
%       P - camera matrices (K-cell of 3x4 matrices), is converted to a
%           3x4xK matrix in the code.
%       image_sizes - 2xK matrix of image sizes
%   OUTPUTS:
%       X - 3D point that is MLE given the K projections in different
%           images
%
% We do some preprocessing at the start of the code to ensure stability at
% runtime. You don't have to modify it.
%
% Using, the linear estimate as a starting value, simply keep doing newton 
% steps (we do a fixed amount of 10 for speed)

if iscell(P)
  P = cat(3,P{:});
end
K = size(P,3);

% Get the linear estimate as the initial value
X = linearEstimate3D(x,P,image_sizes);
X = X(1:3)/X(4);

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

% TODO: Implement the Newton Step functionality below

for i = 1:10
  [e J] = reprojectionError(X, x, P);
  X = X - (J' * pinv(J*J') * e);
end

X = [X; 1];

end
