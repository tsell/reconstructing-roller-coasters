function [ frame ] = createFrame( matches, focal_length, F, im_size )
%CREATEFRAME creates a frame struct
%  A frame struct holds the data between correspondences between two
%  images. It contains the following data:
%     (1) Focal Length
%     (2) Image size associated with the camera
%     (3) Matches: a 4xP matrix of P pairs of 2D locations of matches in the
%         two images. We use this to generate:
%           - match_points: 2xQ matrix of all locations of Q total matched points
%           - match_idx: MxR matrix that where the (i,j) element denotes
%                  whether an camera i has seen point j. If so, it gives the
%                  index in match_values of that point. Columns with
%                  multiple nonzero indices mean that that point is visible
%                  in multiple cameras.
%     (4) Intrisic Camera Matrix
%     (5) Motion: a 3x4xM matrix of M transformations from the frame's
%         coordinate system to each of the camera's coordinate systems
%     (6) Structure: a 3xN matrix of N reconstructed 3D points
%
% Arguments:
%          matches  - a 4XD matrix of pairs of 2D locations (each pair is
%               vertically stacked)
%          focal length - the focal length is assumed to be constant for
%               all cameras
%          Fundamental matrix between two camera views
%          Image size
%
% Note that this method assumes an initialization of a two camera correspondence,
% but frames can represent more cameras. See the mergeAllFrames() function
% for more details on how to do so
%
% Returns:
%          frame - Creates a new frame struct and outputs it

frame.focal_length = focal_length;
frame.im_size = im_size;
frame.matches = matches;

% Using the matches, 
nPts = size(matches,2);
frame.match_idx = sparse([1:nPts; nPts + (1:nPts)]);
frame.match_points = [matches(1:2,:) matches(3:4,:)];

% create intrinsic matrix
frame.K = eye(3); frame.K(1,1) = focal_length; frame.K(2,2) = focal_length;

% form essential matrix to extract transformation between two cameras
E = frame.K'*F*frame.K;
frame.T = computeRTFromE(E, matches, frame.K, im_size(2), im_size(1));

% a simple initialization of motion values is that one camera is at the
% origin while the other is set to the transformation between the two
% cameras
frame.motion(:,:,1) = [eye(3) [0;0;0]];
frame.motion(:,:,2) = frame.T;

% Triangulate the 3d points from the matches
% This function sets the structure variable
frame = triangulate(frame);


end

