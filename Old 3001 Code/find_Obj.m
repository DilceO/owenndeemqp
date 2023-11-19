function centroid = find_Obj(I,color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
NHOOD = [0 1 0; 1 1 1; 0 1 0];
SE = strel('arbitrary',NHOOD);

switch color
    case "red"
        mask = Red_Mask(I);

    case "green"
        mask = Green_Mask(I);
    case "yellow"
        mask = Yellow_Mask(I);
    case "orange"
        mask = Orange_Mask(I);
    case "blue"
        mask = Blue_Mask(I);
    otherwise
        disp("Please pick a valid color...")
end

mask = imerode(mask,SE);
mask = edge(mask,'sobel');
mask = imfill(mask, 'holes');        % this bricks the proram
centroid = regionprops(mask, 'centroid');  
end