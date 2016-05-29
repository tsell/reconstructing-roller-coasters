function frame = triangulate(frame)
%TRIANGULATE given pairs of matching points, finds their 3D locations and
% stores them in structure
% Usage:   frame = triangulate(frame)
%
% Arguments:
%          frame  - a frame structure (see createFrame function for more
%          details)
%
% Note that this method can be used to initialize structure or to refine it
% once multiple image correspondences are merged together (see
% mergeAllFrames function)
%
% Returns:
%          frame - The same input frame, but with structure set to the 3D
%          locations

% Define the number of matching points across all views
nPts = size(frame.matches,2);
if isfield(frame,'structure')
    nPts = size(frame.structure,2);
end

X = zeros(4,nPts);

% For each of the matching points, we find every camera that has it in view
% and its coordinate in the image frame. Then we find the 3D point that
% minimizes the algebraic distance when projected to each of the cameras
for i=1:nPts
    validCamera = find(full(frame.match_idx(:,i)~=0))';

    P=cell (1,length(validCamera));
    x=zeros(2,length(validCamera));
    cnt = 0;
    
    for c=validCamera
        cnt = cnt + 1;
        % x (2-by-K matrix)
        x(:,cnt) = frame.match_points(:,frame.match_idx(c,i));
        
        % P (K-cell of 3-by-4 matrices)
        P{cnt} = frame.K * frame.motion(:,:,c);
    end
    X(:,i) = nonlinEstimate3D(x,P,repmat([frame.im_size(2); frame.im_size(1)],1,length(P)));
    
end

% The result must be normalized by the last homogeneous coordinate.
frame.structure = X(1:3,:) ./ X([4 4 4],:);