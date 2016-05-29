function Rt =computeRTFromE(E, matches, K, im_width, im_height)
%BUNDLEADJUSTMENT runs bundle adjustment on a group of cameras to adjust
% motion and structure
% Arguments:
%          E  - The Essential Matrix between two cameras
%          matches - the matches between two cameras
%          K - camera matrix
%          im_width - image width in pixels
%          im_height - image height in pixels
% Returns:
%          Rt - 3x4 Matrix where first 3x3 is R, last 3x1 is T
%
% Initially, for part A, you will return 4 of them, but your final result
% should only be one 3x4 Matrix. Look at pages 257-259 in Multiple View
% Geometry by Hartley and Zisserman for implementation details.

%TODO: FILL IN THE FUNCTION BELOW
[U,D,V] = svd(E);
W = [0 -1 0; 1 0 0; 0 0 1];
Z = [0 1 0;-1 0 0; 0 0 0];
% translaton vector (skew-symmetry matrix)
S = U*Z*U';
% two possible rotation matrices
R1 = U*W*V';
R2 = U*W*V';
% translation vector
T = U(:,3);
% check determinant
if det(R1) < 0
  R1 = -R1;
end
if det(R2) < 0
  R2 = -R2;
end
P{1} = K*[eye(3) [0 0 0]'];
Rt{1} = [R1 T];
Rt{2} = [R2 T];
Rt{3} = [R1 -T];
Rt{4} = [R2 -T];
num_pos = zeros(4, 1);
for i = 1:4
  P{2} = K*Rt{i};
  for j = 1:size(matches, 2)
    pt = nonlinEstimate3D(reshape(matches(:,1),2,2),P,repmat([im_height;im_width],1,2));
    pt = pt / pt(4);
    if pt(3) > 0 && (Rt{i}(3,1:3) * ((pt(1:3) - Rt{i}(1:3,4)))) > 0
      num_pos(i) = num_pos(i) + 1;
    end
  end
end
[~, I] = max(num_pos);
Rt = Rt{I};
