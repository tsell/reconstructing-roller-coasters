function errors = reprojectionErrorMotStr(match_idx,match_points,px,py,f,motion,structure)
%REPROJECTIONERRORMOTSTR computes the reprojection error using the motion
%and structure
% Arguments:
%          match_idx: index of KxN for N points observed by K cameras, sparse matrix
%          match_points: 2xM for M matched points, referred by match_idx
%          px,py: princple points in pixels
%          f: optional parameter, focal length in pixels
%          motion: 3x2xK for K cameras, represented in angle axis form
%          structure: 3xN for N points
%
% Returns:
%          errors - 2xK for the reprojection error for each of K cameras

nCam = size(match_idx,1);

% Because of the variable we use to in lsqnonlin, we need to reformat the
% structure and motion first
if nargin==5
    cut = 1+3*2*nCam;
    f = f(1);
    structure = reshape(f(cut+1:end),3,[]);
    motion = reshape(f(2:cut),3,2,[]);
elseif nargin==6
    cut = 3*2*nCam;
    structure = reshape(motion(cut+1:end),3,[]); 
    motion = reshape(motion(1:cut),3,2,[]);
end

motion = reshape(motion,3,2,[]);
structure = reshape(structure,3,[]);

errors = [];
for c=1:nCam
    validPts = match_idx(c,:)~=0;
    validIdx = match_idx(c,validPts);
    
    RP = AngleAxisRotatePts(motion(:,1,c), structure(:,validPts));
    
    TRX = RP(1,:) + motion(1,2,c);
    TRY = RP(2,:) + motion(2,2,c);
    TRZ = RP(3,:) + motion(3,2,c);
    
    TRXoZ = TRX./TRZ;
    TRYoZ = TRY./TRZ;
    
    x = f*TRXoZ + px;
    y = f*TRYoZ + py;
    
    ox = match_points(1,validIdx);
    oy = match_points(2,validIdx);
    
    errors = [errors [x-ox; y-oy]];
end

errors = errors(:);
