function T = translMatrix(translation)
% TRANSLMATRIX Generate a 3x3 homogeneous translation matrix (2D)
%   T = TRANSLMATRIX([x y]) returns the 3x3 homogeneous translation matrix
%   for a translation along x and y. Works with numeric or symbolic inputs.
%
%   Input:
%       translation - row vector [x y] (double or sym)
%
%   Output:
%       T - 3x3 homogeneous translation matrix (double or sym)
%
%   Example:
%       % Numeric:
%       T1 = translMatrix([4 2]);
%
%       % Symbolic:
%       syms x y
%       Ts = translMatrix([x y]);
%
%   See also: rotMatrix

    if nargin < 1 || numel(translation) ~= 2
        error('Input must be a row vector [x y].');
    end

    x = translation(1);
    y = translation(2);

    % Numeric literals auto-upcast to sym if x or y are symbolic
    T = [ 1 0 x;
          0 1 y;
          0 0 1 ];
end
