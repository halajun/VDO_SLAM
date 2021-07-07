%% script for color cubes segmentation in Oxford Multi-motion Dataset (https://robotic-esp.com/datasets/omd/)

% setthings
dataset = 'swinging_4_unconstrained';
display = 1;
saveOutput = 1;

% path
folder_path = '/omd/swinging_4_unconstrained_stereo/';
stereo_path = strcat(folder_path, '/image_0/');
segmentation_path = strcat(folder_path, '/mask/');

file_pattern = fullfile(stereo_path, '*.png');
rgb_files = dir(file_pattern);
figure; 
for i = 1:length(rgb_files)
    i
    base_frame_name = rgb_files(i).name;
    full_frame_name = fullfile(stereo_path, base_frame_name);
    
    %read rgb image
    I_rgb = imread(strcat(full_frame_name));
    
    %foreground-background segmentation -Otsu's method
    I_gray = rgb2gray(I_rgb);
    T = graythresh(I_gray);
    mask_fore_back = I_gray>0.78*T.*255;
    
    %hsv color segmentation
    I = I_rgb.*repmat(uint8(mask_fore_back),[1,1,3]);
    I_hsv = rgb2hsv(I);
    mask_hsv = I_hsv(:,:,2)>0.28;
    
    masked_I = I_rgb.*repmat(uint8(mask_hsv),[1,1,3]);
    
    se = strel('cube',9);
    eroded_mask_hsv = imerode(mask_hsv,se);
    
    se = strel('cube',38);
    dilated_mask_hsv = imdilate(eroded_mask_hsv,se);
    
    %post process - will give great mask, multiple labels per mask
    %final_mask = dilated_mask_hsv & mask_hsv;
    %labels = bwlabel(final_mask);
    
    %post process - will give great mask, multiple labels per mask
    labels = bwlabel(dilated_mask_hsv);
    
    uv = unique(labels);
    counts  = histc(reshape(labels,1,size(labels,1)*size(labels,2))',uv);
    
    for k=1:size(uv,1)
        if (counts(k)<10000)
            labels(labels == uv(k)) = 0;
        end
    end
    
    % uv_new = unique(labels)'
    
    if saveOutput
        dlmwrite(strcat(segmentation_path,base_frame_name(1:end-4),'_labels.txt'),labels,'delimiter','\t','newline','pc');
    end
    
    if display
        labelled_I = labeloverlay(double(mask_hsv),labels);
%         figure; imshow(I_rgb); title('original RGB image')
%         figure; imshow(mask_fore_back); title('foreground-background segmentation - Otsu Method')
%         figure; imshow(mask_hsv); title('cube segmentation - HSV color thresholding')
%         figure; imshow(masked_I); title('segmented image')
%         figure; imshow(eroded_mask_hsv); title('eroded mask')
%         figure; imshow(dilated_mask_hsv); title('dilated mask')
        imshow(labelled_I); title('labelled mask')
    end
end