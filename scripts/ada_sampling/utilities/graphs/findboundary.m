function [edge,powercenter] = findboundary(t,powercenter)
tt = t';
tccw = t(:,[3 1 2]);
tccw = tccw';
tidx = repmat((1:size(t,1))',[1 3]);
tidx = tidx';
edge = [tt(:) tccw(:)];
edge(edge(:,1)>edge(:,2),:) = [edge(edge(:,1)>edge(:,2),2) edge(edge(:,1)>edge(:,2),1)];
powercenter = powercenter(tidx(:),:);
end