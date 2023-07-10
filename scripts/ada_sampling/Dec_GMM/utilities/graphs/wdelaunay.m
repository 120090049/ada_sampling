function Tri = wdelaunay(X,Y,wts)
X = X(:);
Y = Y(:);

LiftingPoint = [X Y X.^2+Y.^2-wts];

ConvexHull = convhulln(LiftingPoint);

CenterPoint = mean(LiftingPoint,1);

Normals = zeros(size(ConvexHull));
MiddlePoint = zeros(size(ConvexHull));

for i=1:size(ConvexHull,1)
    normal = null(bsxfun(@minus, LiftingPoint(ConvexHull(i,1),:), ...
        LiftingPoint(ConvexHull(i,2:3),:)))';
    if size(normal,1) > 1
        error('nullspace error')
    else
        Normals(i,:) = normal;
    end
    MiddlePoint(i,:) = mean(LiftingPoint(ConvexHull(i,:),:),1);
end

dot = sum(bsxfun(@minus, CenterPoint, MiddlePoint).*Normals, 2);

inidx = dot>0;
Normals(inidx,:) = -Normals(inidx,:);

Tri = ConvexHull(Normals(:,end)<0, :);
end