length = 45;
width = 21;
[Xss, ksx_g, ksy_g] = generate_coordinates(length, width);

function [Xss, ksx_g, ksy_g] = generate_coordinates(length, width)
    y = repmat(0:length-1, 1, width);
    x = repmat(0:width-1, length, 1);
    x = x(:)';
    y = y(:)';
    Xss = [x; y]';
    
    ksx_g = repmat(0:width-1, length, 1);
    ksy_g = repmat(0:length-1, width, 1)';
end
