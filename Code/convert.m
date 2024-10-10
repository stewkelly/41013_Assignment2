% MATLAB Script to Convert Grayscale Image to RGB and Save It

% Step 1: Read the Image
inputFilename = 'tile.jpg';        % Input grayscale image filename
outputFilename = 'tile_rgb.jpg';   % Output RGB image filename

% Read the image using imread
cdata = imread(inputFilename);

% Step 2: Check if the Image is Grayscale
% Grayscale images have only two dimensions (MxN) or the third dimension is size 1
if ndims(cdata) == 2 || (ndims(cdata) == 3 && size(cdata, 3) == 1)
    fprintf('The image "%s" is detected as Grayscale.\n', inputFilename);
    
    % Step 3: Convert Grayscale to RGB
    % Replicate the grayscale data across the three RGB channels
    rgbImage = repmat(cdata, [1, 1, 3]);
    
    % Optional: Display the original and converted images
    figure;
    subplot(1,2,1);
    imshow(cdata);
    title('Original Grayscale Image');
    
    subplot(1,2,2);
    imshow(rgbImage);
    title('Converted RGB Image');
    
    % Step 4: Save the RGB Image
    imwrite(rgbImage, outputFilename);
    fprintf('Converted RGB image saved as "%s".\n', outputFilename);
    
else
    fprintf('The image "%s" is already an RGB image. No conversion needed.\n', inputFilename);
    
    % Optional: Display the RGB image
    figure;
    imshow(cdata);
    title('Original RGB Image');
    
    % If you still want to save a copy, uncomment the following line:
    % imwrite(cdata, 'copy_of_tile_rgb.jpg');
end
