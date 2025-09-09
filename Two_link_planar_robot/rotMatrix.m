function R = rotMatrix(theta, option)
% ROTMATRIX Generate a 3x3 homogeneous rotation matrix (2D)
%   R = ROTMATRIX(theta) returns the 3x3 homogeneous rotation matrix
%   for an angle theta given in radians. Works with numeric or symbolic.
%
%   R = ROTMATRIX(theta, 'deg') interprets theta in degrees.
%
%   Input:
%       theta  - rotation angle (radians by default) [double or sym]
%       option - (optional) 'deg' if theta is in degrees
%
%   Output:
%       R - 3x3 homogeneous rotation matrix (double or sym)
%
%   Example:
%       % Numeric:
%       R1 = rotMatrix(45, 'deg');
%       R2 = rotMatrix(pi/2);
%
%       % Symbolic:
%       syms t
%       Rs = rotMatrix(t);          % radians
%       Rs_deg = rotMatrix(sym(30),'deg'); % degrees
%
%   See also: translMatrix

    if nargin > 1 && strcmpi(option, 'deg')
        % Use symbolic-safe degree->radian conversion
        theta = theta * pi/180;
    end

    % cos/sin accept both double and sym
    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
         0,           0,          1];
end
