% %%%%%%%%%%%%%%%%%%%%%%
videoWriteEnable=0;
if(videoWriteEnable)
    videoWriteHandler=VideoWriter('featureDetTrace00.avi','Motion JPEG AVI');
    videoWriteHandler.FrameRate=10;
    open(videoWriteHandler);
end

% %%%%%%%%%%%%%%%%%%%%%%
load("camParams.mat");

% %%%%%%%%%%%%%%%%%%%%%%
videoReadFilesName=[...
"./movie/2021_07_11_15_20_48_Gsn.AVI";
"./movie/2021_07_11_16_30_44_Gsn.AVI";
"./movie/2021_07_18_09_00_45_Gsn.AVI";
"./movie/2021_07_18_14_14_20_Gsn.AVI";
"./movie/2021_07_11_15_21_19_Gsn.AVI";
"./movie/2021_07_17_15_35_06_Gsn.AVI";
"./movie/2021_07_18_09_01_16_Gsn.AVI";
"./movie/2021_07_18_15_59_49_Gsn.AVI";
"./movie/2021_07_11_16_30_13_Gsn.AVI";
"./movie/2021_07_17_15_35_37_Gsn.AVI";
"./movie/2021_07_18_14_13_50_Gsn.AVI";
"./movie/2021_07_18_16_00_20_Gsn.AVI"];
videoReadFilesNameSize=size(videoReadFilesName,1);


%for i=1:videoReadFilesNameSize
for i=5:5

    videoReadFileName=videoReadFilesName(i,:);
    video=VideoReader(videoReadFileName);    
    [imageColorPrv,imageGrayPrv]=readFrameColorGray(video);
    imagePts=detectFeatures(imageGrayPrv);
    [featPrv,ptsPrv]=extractFeatures(imageGrayPrv,imagePts);    

    logIdx=1;
    
    logLoc=zeros(video.NumFrames+1,3);

    while( hasFrame(video) )
        logIdx=logIdx+1;

        % Get image 
        [imageColorCur,imageGrayCur]=readFrameColorGray(video);

        % Get feature
        framePts = detectFeatures(imageGrayCur);        
        [featCur,ptsCur] = extractFeatures(imageGrayCur,framePts);

        % Matching
        idxPairs=matchFeatures(featPrv,featCur,"Unique",true);

        % Select Matched point
        idxParisSize=size(idxPairs);
        idxPairsFull=zeros(idxParisSize);
        ptsCur_Location = ptsCur.Location;
        ptsPrv_Location = ptsPrv.Location;        
        lastIdx=0;
        for k=1:idxParisSize(1)
            idxPrv=idxPairs(k,1);
            idxCur=idxPairs(k,2);
            ptPrvLoc=ptsPrv_Location(idxPrv,:);
            ptCurLoc=ptsCur_Location(idxCur,:);
            ptDiffLoc=norm(ptCurLoc-ptPrvLoc);
            if( ptDiffLoc < 100 && ptCurLoc(1,2)<760 && ptPrvLoc(1,2)<760 )
                lastIdx=lastIdx+1;
                idxPairsFull(lastIdx,:)=idxPairs(k,:);
            end
        end
        idxSelectedPairs=idxPairsFull(1:lastIdx,:);
        matchedPtsPrv = ptsPrv( idxSelectedPairs(:,1) );
        matchedPtsCur = ptsCur( idxSelectedPairs(:,2) );
        
        %
        matchedPtsPrvLoc = matchedPtsPrv.Location;
        matchedPtsCurLoc = matchedPtsCur.Location;

        [E,inliersIndexE] = estimateEssentialMatrix(matchedPtsPrvLoc,matchedPtsCurLoc,camParams);
        [relOri,relLoc]   = relativeCameraPose(E,camParams,matchedPtsPrvLoc,matchedPtsCurLoc);
        
        %
        tmp=(logLoc(1:(logIdx-1),1:3)-relLoc)*relOri';
        logLoc(1:(logIdx-1),:)=tmp;
        

        % Show Image
        figure(1);
        showMatchedFeatures(imageGrayPrv,imageGrayCur,matchedPtsPrv,matchedPtsCur,'blend');
        % Show Log
        figure(2);
        plot3(logLoc(1:logIdx,1),logLoc(1:logIdx,2),logLoc(1:logIdx,3),'.-');
        axis equal;
        

        % Copy Cur2Prv
        imageGrayPrv = imageGrayCur;
        featPrv      = featCur;
        ptsPrv       = ptsCur;

        % Video Write
        if(videoWriteEnable )
            frame = getframe(gcf);
            writeVideo(videoWriteHandler,frame);
        end        
    end
end

% 
if(videoWriteEnable)
    close(videoWriteHandler);
end


%% %%%%%%%%%%%%%%%%%%%%%%
function Pts=detectFeatures(I)
    mode =1 ;
    switch(mode)
        case 1
            Pts = detectSURFFeatures(I);
        case 2
            Pts = detectORBFeatures(I);
    end
end

function [imageColor,imageGray]=readFrameColorGray(video)
    imageColor = readFrame(video);
    imageGray  = rgb2gray(imageColor);
end
