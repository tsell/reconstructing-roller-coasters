function mergedFrame = mergeAllFrames(frames)
%MERGEALLFRAMES Given a cell array of frames, merges them into one frame
% Arguments:
%          frames  - a cell array of frames (see createFrame() for frame
%               details)
%
% Returns:
%          mergedFrame - A single frame that has consolidated the structure
%               and motion of all the cameras and 3D points

    mergedFrame = frames{1};
    for i=2:length(frames)
        % merge each new frame into our current running frame
        mergedFrame = mergeTwoFrames(mergedFrame, frames{i}, i);
        
        % update the 3D points in structure once motion is set
        mergedFrame = triangulate(mergedFrame);
        [mergedFrame.motion, mergedFrame.structure, f] = bundleAdjustment(mergedFrame);

        % outlier rejection
         mergedFrame = removeOutliers(mergedFrame, 10);

        % bundle adjustment
         [mergedFrame.motion, mergedFrame.structure, f] = bundleAdjustment(mergedFrame);   
    end
    
end

% This is a helper function that merges one frame into another
function mergedFrame = mergeTwoFrames(frameA, frameB, len)
    mergedFrame = frameA;
    % transform frameB.motion and frameB.structure to be in the same world coordinate system of frameA
    frameBto1 = multiplyTransformations(inverse(frameA.motion(:,:,end)), frameB.motion(:,:,1));
    frameB.structure = transformPoints(frameB.structure, frameBto1);
    for i=1:2
        frameB.motion(:,:,i) = multiplyTransformations(frameB.motion(:,:,i), inverse(frameBto1));
    end

    % since camera is in the merged reference frame, add it to our motion
    % matrix
    mergedFrame.motion(:,:,len+1) = frameB.motion(:,:,2);

    % The tricky part here is that we now have to reconcile the matched
    % points to generate the structure. We must merge the new matched_points
    % from each additional frame, but must associate points that correspond
    % to already seen points in the same column.
    trA = find(frameA.match_idx(end,:)~=0);
    xyA = frameA.match_points(:,frameA.match_idx(end,trA));

    trB = find(frameB.match_idx(1,:)~=0);
    xyB = frameB.match_points(:,frameB.match_idx(1,trB));

    [xyCommon,iA,iB] = intersect(xyA',xyB','rows');
    xyCommon = xyCommon';

    % Here we add a new point to match_points and put that index in the
    % same column in match_idx, as it has been already seen 
    for i=1:size(xyCommon,2)
        idA = trA(iA(i));
        idB = trB(iB(i));

        B_match_idx = frameB.match_idx(2,idB);

        mergedFrame.match_points(:,end+1) = frameB.match_points(:,B_match_idx);  
        mergedFrame.match_idx(len+1,idA) = size(mergedFrame.match_points,2);            
    end

    % One of the cameras in frame B is the same as in frame A
    % We will add all new points from this camera into the match fields
    [xyNewFromB, iB] = setdiff(xyB',xyA','rows');
    xyNewFromB = xyNewFromB';

    for i=1:size(xyNewFromB,2)
        idB = trB(iB(i));

        mergedFrame.match_points(:,end+1) = frameB.match_points(:,frameB.match_idx(1,idB));
        mergedFrame.match_idx(len,end+1) = size(mergedFrame.match_points,2);       
        mergedFrame.structure(:,end+1) = frameB.structure(:,idB);

        B_match_idx = frameB.match_idx(2,idB);

        mergedFrame.match_points(:,end+1) = frameB.match_points(:,B_match_idx);
        mergedFrame.match_idx(len+1,end) = size(mergedFrame.match_points,2);            

    end    

    % The other camera in frame B is new
    % We can simply add all the new points from here
    newB = false(1,2);
    newB(2) = true;

    tr2add = sum(frameB.match_idx(~newB,:)~=0,1)==0 & sum(frameB.match_idx(newB,:)~=0,1)>0;

    if any(tr2add)

        ids = full(frameB.match_idx(indexNewFramesFromB,tr2add));

        curValCnt = size(mergedFrame.match_points,2);
        nonZerosID = find(ids(:)>0);

        mergedFrame.match_points(:,(curValCnt+1):(curValCnt+length(nonZerosID))) = frameB.match_points(:,ids(nonZerosID));

        idsNew = zeros(size(ids));
        idsNew(nonZerosID) = (curValCnt+1):(curValCnt+length(nonZerosID));

        mergedFrame.match_idx(length(frameA.frames)+1:end,size(mergedFrame.match_idx,2)+1:size(mergedFrame.match_idx,2)+size(idsNew,2)) = sparse(idsNew);

        mergedFrame.structure(:,size(mergedFrame.match_idx,2)+1:size(mergedFrame.match_idx,2)+size(idsNew,2)) = frameB.structure(:,tr2add);
    end

    return;
end


% Some easy helper functions to do quick transformation functions
function T = multiplyTransformations(A, B)
T = [A(:,1:3)* B(:,1:3)  A(:,1:3)*B(:,4) + A(:,4)];
end

function out = inverse(in)

out = [in(1:3,1:3)', - in(1:3,1:3)'* in(1:3,4)];
end

function result = transformPoints(points3d, Rt, isInverse)

if nargin<3 || ~isInverse
    result = Rt(:,1:3) * points3d + repmat(Rt(:,4),1,size(points3d,2));
else
    result = Rt(:,1:3)' * (points3d - repmat(Rt(:,4),1,size(points3d,2)));
end
end