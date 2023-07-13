length = 200;
width = 100;
[Xss, ksx_g, ksy_g] = generate_coordinates(length, width);

A = zeros(5, 2);  % 创建一个全零数组

increment = 1;  % 每个元素递增的值

for j = 1:size(A, 2)
    for i = 1:size(A, 1)
        A(i, j) = A(i, j) + increment;
        increment = increment + 1;
    end
end

A

B = reshape(A, [], 1);

B
function [Xss, ksx_g, ksy_g] = generate_coordinates(length, width)
    [X, Y] = meshgrid(0:length-1, 0:width-1);
    Xss = [X(:), Y(:)];
    ksx_g = repmat(0:width-1, length, 1);
    ksy_g = repmat(0:length-1, width, 1)';
end
