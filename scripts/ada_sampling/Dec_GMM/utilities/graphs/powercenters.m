function [PowerCenter, powers] = powercenters(Tri,X,Y,wts)
% function [PC, powers] = powercentersPD(T, E, wts)
%
% T: triangulation
% E: set of points
% wts: weights of points in E
%
% The output array PC contains the power centers of the triangles in T.
% That is, row i of PC contains the point that is equidistant from each
% vertex of the triangle specified by row i of T, with respect to the power
% of each vertex. The output array powers contains the power of each power
% center.

[N, ~] = size(Tri);
PowerCenter = zeros(N,2);
powers = zeros(N,1);
X = X(:);
Y = Y(:);
for i=1:N
    x = X(Tri(i,:),:);
    y = Y(Tri(i,:),:);
    w = wts(Tri(i,:));
    Ac = 2*([x(2:3) y(2:3)] - repmat([x(1) y(1)],[2 1]));
    bc = x(2:3).^2+y(2:3).^2-w(2:3)-repmat(x(1)^2+y(1)^2-w(1),[2 1]);
    pc = Ac\bc;
    PowerCenter(i,:) = pc;
    powers(i) = norm(pc - [x(1);y(1)])^2 - w(1);
end