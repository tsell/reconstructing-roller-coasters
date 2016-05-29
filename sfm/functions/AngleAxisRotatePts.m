function result = AngleAxisRotatePts(angle_axis, pt)

angle_axis = reshape(angle_axis(1:3),1,3);

theta2 = dot(angle_axis,angle_axis);

if (theta2 > 0.0)
    % Away from zero, use the rodriguez formula
    %
    %   result = pt costheta + (w x pt) * sintheta + w (w . pt) (1 - costheta)
    %
    % We want to be careful to only evaluate the square root if the
    % norm of the angle_axis vector is greater than zero. Otherwise
    % we get a division by zero.
    
    theta = sqrt(theta2);
    w = angle_axis / theta;
    
    costheta = cos(theta);
    sintheta = sin(theta);
    
    % w_cross_pt = cross(w, pt);
    w_cross_pt = xprodmat(w) * pt;
    
    %w_dot_pt = dot(w, pt);
    w_dot_pt = w * pt;
    
    result = pt * costheta + w_cross_pt * sintheta + (w' * (1 - costheta)) * w_dot_pt;
    
    
else
    % Near zero, the first order Taylor approximation of the rotation
    % matrix R corresponding to a vector w and angle w is
    %
    %   R = I + hat(w) * sin(theta)
    %
    % But sintheta ~ theta and theta * w = angle_axis, which gives us
    %
    %  R = I + hat(w)
    %
    % and actually performing multiplication with the point pt, gives us
    % R * pt = pt + w x pt.
    %
    % Switching to the Taylor expansion at zero helps avoid all sorts
    % of numerical nastiness.
    
    %w_cross_pt = cross(angle_axis, pt);
    w_cross_pt = xprodmat(angle_axis) * pt; % vectorize version
    
    result = pt + w_cross_pt;
end
    


function A=xprodmat(a)
%Matrix representation of a cross product
%
% A=xprodmat(a)
%
%in:
%
% a: 3D vector
%
%out:
%
% A: a matrix such that A*b=cross(a,b)


if length(a)<3, error 'Input must be a vector of length 3'; end

ax=a(1);
ay=a(2);
az=a(3);

A=zeros(3);

A(2,1)=az;  A(1,2)=-az;
A(3,1)=-ay; A(1,3)=ay;
A(3,2)=ax;  A(2,3)=-ax;

