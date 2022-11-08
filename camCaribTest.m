videoFiles=["./carib_movie/2022_11_05_12_21_30_Nor.AVI";
            "./carib_movie/2022_11_05_12_22_00_Nor.AVI"]; 

mkdir("./carib_image");

capList = [1, 8.0;
           1, 9.0;
           1,10.0;
           1,11.0;
           1,12.0;
           1,13.0;
           1,14.0;
           1,17.0;
           1,18.0;
           1,19.0;
           1,20.0;
           %1,21.0;
           1,23.0;
           1,24.0;
           1,28.0;
           %1,28.4;
           2, 0.9;
           2,12.0;
           2,17.0;
           2,19.0];

%caplist = [2,19.0];
capListSize=size(capList,1);

%get boardSize
j=1;
i=capList(j,1);
video=VideoReader(videoFiles(i,:));
video.CurrentTime=capList(j,2);
img = readFrame(video);
[imagePoint,boardSize,imgUsed] = detectCheckerboardPoints(img,'HighDistortion',true);
squareSizeInMM=30;
worldPoint  = generateCheckerboardPoints(boardSize,squareSizeInMM);
imagePoints = zeros(size(worldPoint,1),size(worldPoint,2),capListSize);

figure(1);
for j=1:size(capList,1)
    i=capList(j,1);
    video=VideoReader(videoFiles(i,:));    
    video.CurrentTime=capList(j,2);
    img = readFrame(video);
    
    imshow(img);
    drawnow;
    if(1)
        [imagePoint,boardSize,imgUsed] = detectCheckerboardPoints(img,'HighDistortion',true);
        if(size(imagePoint,1)>1)
            mon = insertMarker(img, imagePoint(:,:), 'o', 'Color', 'red', 'Size', 10);
            imshow(mon);
            imagePoints(:,:,j)=reshape(imagePoint,[size(worldPoint,1),size(worldPoint,2),1]);
            drawnow;                
            imwrite(img,sprintf("./carib_image/img%02d.png",j));
        else
            imshow(img);
        end
    end
end

[camParams] = estimateCameraParameters(imagePoints,worldPoint);  
figure(2);
showReprojectionErrors(camParams);
figure(3);
showExtrinsics(camParams);

save("camParams.mat","camParams");
