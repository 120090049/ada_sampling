function K =  powercellidx(X,Y,x,y)
X = X(:);
Y = Y(:);
x = x(:);
y = y(:);
% wts = wts(:);

Distance = pdist2([x y],[X Y],'euclidean');
wDistance = Distance.^2; % - repmat(wts',[length(x) 1]);
[~,K] = min(wDistance,[],2);

end