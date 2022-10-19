%% Libraly Setting
addpath('../../pllib');

%% Video write setting
video_enable=0;
if(video_enable)
    video_handler=VideoWriter('roadSfMwoScaleEstimation.avi','Motion JPEG AVI');
    %video_handler=VideoWriter('testTraceModel5.avi','Motion JPEG 2000');
    %video_handler.FrameRate=15*4;
    video_handler.FrameRate=10;
    open(video_handler);
end


%% Figure setting
for i=1:3
    figure(i);
    clf;
end

%% Sampling parmaeter 
Fs=10;
Ts=1/Fs;

%% Set course
deltaStep=60/3.6/Fs;
shapeR=100;
shapeStraightLeng=50;
if(0)
    shapeTh=deg2rad((0:-1:-45)');
    shapeCurve=[shapeR*sin(shapeTh),shapeR-shapeR*cos(shapeTh)];
    shapeStraight1=[shapeStraightLeng,0];
    shapeStraight2=(shapeCurve(end,:)-shapeCurve(end-1,:))/deltaStep*shapeStraightLeng+shapeCurve(end,:);
    shape=[shapeStraight1;shapeCurve;shapeStraight2];
    shape=getShapeConcatinate(shape,shape*[1,0;0,-1]);
elseif(0)
    shape1=getShapeStraight(-shapeStraightLeng,0);
    shape2=getShapeCurve(-shapeR,deg2rad(45));
    shape3=getShapeCurve( shapeR/2,deg2rad(60));
    shape4=getShapeCurve(-shapeR/2,deg2rad(60));
    shape =getShapeConcatinate(shape1,shape2);
    shape =getShapeConcatinate(shape,shape3);
    shape =getShapeConcatinate(shape,shape4);
    shape =getShapeConcatinate(shape,shape1);
elseif(1)
    shape=getShapeStraight(-shapeStraightLeng,0);
    for i=1:10
        shapeR = (40 + randn(1,1)*20)*sign(randn(1,1));
        shapeTh=  30 + randn(1,1)*15;
        shape_curve = getShapeCurve(shapeR,deg2rad(shapeTh));
        shape =getShapeConcatinate(shape,shape_curve);
    end
end

trajectoryXY=getChopped(shape,deltaStep);
trajectoryXYYaw=appendYaw(trajectoryXY);
trajectorySize=size(trajectoryXY,1);
trajectoryZ = zeros(trajectorySize,1);
trajectory=[trajectoryXY(:,1:2),trajectoryZ,trajectoryXYYaw(:,3)+pi];

sfmPos=zeros(trajectorySize,6);
estTrj=zeros(trajectorySize,6);

lineSize=trajectorySize;
lineOfsR=-2.75/2*ones(lineSize,1)+randn(lineSize,1)*0.0;
lineOfsL=+2.75/2*ones(lineSize,1)+randn(lineSize,1)*0.0;
lineZ  =zeros(lineSize,1)+randn(lineSize,1)*0.0001;
line1XY=getOfsPath(trajectoryXY,lineOfsR);
line2XY=getOfsPath(trajectoryXY,lineOfsL);
line1=[line1XY,lineZ];
line2=[line2XY,lineZ];



%% Set world parameter

world_w =2.75/2;
world_h =0;
world_d0=5;
world_d1=10;

camZ=1.2;

roadmark=[+world_d0,-world_w,world_h;
          +world_d0,+world_w,world_h;
          +world_d1,+world_w,world_h;
          +world_d1,-world_w,world_h;
          +world_d0,-world_w,world_h];

paint_param=getPlaneParam(roadmark(1,1:3),...
                          roadmark(2,1:3),...
                          roadmark(3,1:3));


%%
if(1)
        hold off;
        plot3(line1(:,1),line1(:,2),line1(:,3),'-','Color','#C0C0C0');
        hold on;                
        plot3(line2(:,1),line2(:,2),line2(:,3),'-','Color','#FFC0C0');
        plot3(roadmark(:,1),roadmark(:,2),roadmark(:,3),'x-b');
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),':','Color','#004000','LineWidth',1);
        view(0,90);
        axis equal;        
end

%% Set camera setting
u_fov=1;
a_aspect=9/16;
FOV_H=2*pi*120/360;
v_fov=a_aspect*u_fov;
camF=u_fov/tan(FOV_H/2);
camFrame=[-u_fov,+u_fov,-v_fov,v_fov];

%% Get camera calibration

% Create checker borad
figure(2);
meshSizeX=11;
meshSizeY=5;
meshSize=meshSizeY*meshSizeX;
meshNode=zeros(meshSize,3);
for j=1:meshSizeY
    for i=1:meshSizeX
        k=(j-1)*meshSizeX+i;
        meshNode(k,1)=(i-(meshSizeX-1)/2-1)*1+(rand-0.5);
        meshNode(k,2)=(j-(meshSizeY-1)/2-1)*1+(rand-0.5);
        meshNode(k,3)=0;
    end
end

subplot(2,1,1);
plot(meshNode(:,1),meshNode(:,2),'ob',...
     meshNode(:,1),meshNode(:,2),'-r');
axis equal;

frameSize=24;
meshImgs=zeros(meshSize,2,frameSize);

% Set multiple poses 
for k=1:frameSize
    th =(k-1)/(frameSize)*2*pi;
    phi=4*(k-1)/(frameSize)*2*pi;
    Rxy =7+rand*0;
    Rz  =2+rand*0;
    c0pos=[0,0,-2];
    c1pos=[Rxy*cos(th),Rxy*sin(th),4+Rz*sin(phi)];d1pos=c0pos-c1pos;
    rxy2=norm(d1pos(1,1:2));
    %c1dir=[-atan2(d1pos(3),rxy2), -th-pi/2, -pi/2 ];
    c1dir=[-atan2(d1pos(3),rxy2), -atan2(d1pos(2),d1pos(1))+pi/2, -pi/2 ];
    c1=[c1pos,c1dir];    
    meshImg=world2camR(meshNode,camF,c1);
    meshImgs(:,:,k)=reshape(meshImg,[size(meshImg,1),2,1]);
    subplot(2,1,2);    
    plot(meshImg(:,1),meshImg(:,2),'ob',...
         meshImg(:,1),meshImg(:,2),'-r');
    axis ij;
    axis equal;
    axis (camFrame);
    drawnow;
end

% Derive camParams
[camParams] = estimateCameraParameters(meshImgs,meshNode(:,1:2));      

%%  set Parmaeter
antCnt=6;
%% 

init=1;
tm=0;
append_frame_size=6;
frames=[];

vaoid_anotation_exist=0;
prev_anotation_exist =0;

k=1;

camPrevPos=[trajectory(k,1:2),camZ,...
            deg2rad(0),trajectory(k,4),deg2rad(0)];
sfmPos=camPrevPos;

for k=2:trajectorySize

    PitchFreq=0.5;
    PitchAmp =2;
    YawFreq  =0.25;
    YawAmp   =2;
    YawOfs   =0;
    PitchOfs =0;
    th1=deg2rad(PitchAmp*sin(2*pi*PitchFreq*tm)+PitchOfs);
    th2=deg2rad(YawAmp*sin(2*pi*YawFreq*tm)+YawOfs);
    tm=tm+Ts;

    camPos=[trajectory(k,1:2),camZ,...
           -(deg2rad(0)+th1),-(trajectory(k,4)+th2)+pi/2,-deg2rad(0)-pi/2];
    
    [cam_roadmark,camRot,camOfs]=world2camR(roadmark,camF,camPos);
    
    %% Create Frame Wall 
    P2=[           0,camFrame(3),-camF;            
        -camFrame(2),camFrame(3),-camF];
    frame1=getPlaneProjection(P2,camRot,camOfs,paint_param );
    
    wall_p1=frame1(1,1:3);
    wall_p2=frame1(2,1:3);
    wall_p3=[frame1(2,1:2),frame1(2,3)+1];

    P2=[-camFrame(2),camFrame(4),-camF;
        -camFrame(1),camFrame(4),-camF;            
        -camFrame(1),camFrame(3),-camF;
                   0,camFrame(3),-camF];
    wall_param=getPlaneParam(wall_p1,wall_p2,wall_p3);

    frame2=getPlaneProjection(P2,camRot,camOfs,wall_param );
    append_frame=[frame1;frame2];                                 
    frames=[frames;append_frame]; 
    
    %% Append Frame
    append_frame_size=size(append_frame,1);
    pyramid = reshape([append_frame,[append_frame(2:end,:);append_frame(1,:)],repmat(camPos(1,1:3),[append_frame_size,1])]',[3,append_frame_size*3])';
    
    %% Extruct Annotation point
    [cam_q,camPrev_q,refPos,isValid]=getAnotatedPoint(line1, line2, camF,camFrame, camPos,camPrevPos,antCnt);
    if(isValid)    
        %% Derive Vanishment Point
        cam_q0    =getVanishmentPoint(cam_q);
        camPrev_q0=getVanishmentPoint(camPrev_q);
        cam_qc    =[cam_q0    ;cam_q    ];
        camPrev_qc=[camPrev_q0;camPrev_q];
        c1        = sfmPos(k-1,:);
        %% Derive rotation matrix from vanishment point     
        [R1,yawPrev,pitchPrev,rollPrev]=getRotarionMatrixFromVanishmentPointR(camPrev_q0,camF);
        [R2,yaw    ,pitch    ,roll    ]=getRotarionMatrixFromVanishmentPointR(cam_q0    ,camF);
        %[R1,theta,phi,psi]=getRotarionMatrixFromVanishmentPointR(camPrev_q0,camF)
        %[camPrevPos(4),phi]        
        %[R1,theta,phi,psi]=getRotarionMatrixFromVanishmentPoint(cam_q0    ,camF);

        %% Solver
        if(0)
            c2=getNextPos(camPrev_q ,cam_q,...
                          camPrev_q0,cam_q0,...
                          c1,camF,refPos);
        end
        if(1)
            [E,inliersIndexE] = estimateEssentialMatrix(camPrev_q,cam_q,camParams);
            [relOri,relLoc]=relativeCameraPose(E,camParams,camPrev_q,cam_q);
            %[rotMtx,trsVct] = cameraPoseToExtrinsics(relOri,relLoc);   
            cam1Rot= getRotMatrix(c1(4:6));
            %cam1Rot= getRotMatrix(camPrevPos(4:6));
            %cam1Rot=R1;            
            %cam1Rot= getRotMatrix([-yawPrev,-pitchPrev+pi/2,rollPrev-pi/2]);
            %t = [relLoc(1),relLoc(2),relLoc(3)]*cam1Rot'*0.5;            
            t = [relLoc(1),relLoc(2),relLoc(3)]*cam1Rot'*deltaStep*1.0;
            %c2(1,1:3) = c1(1,1:3) + t ;
            %c2(1,4:6) = c1(1,4:6) + 0*[yaw-yawPrev,pitch-pitchPrev,roll-rollPrev];
            %c2(1,1:3)  = camPrevPos(1,1:3) + t ;
            c2(1,1:3)  = c1(1,1:3) + t ;
            c2(1,4:6) = camPrevPos(1,4:6) + [yaw-yawPrev,pitch-pitchPrev,roll-rollPrev];
            %c2(1,4:6) = camPos(1,4:6); 
        end
        if(1) %Use Estimated Value
            sfmPos(k,:)=c2;
        else  %Use Ideal Value
            sfmPos(k,:)=camPos(1,:);
        end
                
    else
        cam_qc    =zeros(5,2);
        camPrev_qc=zeros(5,2);
        sfmPos(k,:)=camPos(1,:);        
    end  

    sfmPosCur=sfmPos(k,:);
    deltaPos = camPos-sfmPosCur;
    
    %sfmPosPrv=sfmPos(k-1,:);    
    %sfmPosCurYaw=camPos(5)-atan2(sfmPosPrv(1,2)-sfmPosCur(1,2),sfmPosPrv(1,1)-sfmPosCur(1,1));
    %R=[ cos(sfmPosCurYaw), sin(sfmPosCurYaw), 0;
    %   -sin(sfmPosCurYaw), cos(sfmPosCurYaw), 0;
    %                    0,                 0, 1]; 
    %estTrj=(sfmPos-sfmPosCur)*R+[camPos(1,1:2),0];
    %sfmPosCur=sfmPos(k,:);

    estTrj=sfmPos+deltaPos-[0,0,camPos(1,3),0,0,0];

    %% Create Cam Frames
    cam_frames     = world2camR(frames,camF,camPos); 
    cam_trajectory = world2camR(trajectory(:,1:3),camF,camPos); 
    cam_line1      = world2camR(line1,camF,camPos); 
    cam_line2      = world2camR(line2,camF,camPos); 
    cam_estTrj     = world2camR(estTrj(1:k,1:3),camF,camPos);
    %cam_sfmPos     = world2camR(sfmPos(1:k,1:3),camF,camPos); 

    %% Create Cam Frames
    camPrev_roadmark   = world2camR(roadmark,camF,camPrevPos);
    camPrev_frames     = world2camR(frames,camF,camPrevPos); 
    camPrev_trajectory = world2camR(trajectory(:,1:3),camF,camPrevPos); 
    camPrev_line1      = world2camR(line1,camF,camPrevPos); 
    camPrev_line2      = world2camR(line2,camF,camPrevPos);
    camPrev_estTrj     = world2camR(estTrj(1:k,1:3),camF,camPrevPos);
    %camPrev_sfmPos     = world2camR(sfmPos(1:k,1:3),camF,camPrevPos); 
    

    %% 
    %drawnow nocallbacks;
    figure(2);
    if(1)
        subplot(2,2,2);
        hold off;    
        plot(cam_frames(:,1),cam_frames(:,2),'.-','Color','#F0C000');
        hold on;
        plot(cam_line1(:,1),cam_line1(:,2),'.-','Color','#C0C0C0','LineWidth',2,'MarkerEdgeColor','#808080','MarkerSize',5);
        plot(cam_line2(:,1),cam_line2(:,2),'.-','Color','#FFC0C0','LineWidth',2,'MarkerEdgeColor','#C08080','MarkerSize',5);
        plot(cam_roadmark(:,1),cam_roadmark(:,2),'x-b');
        plot(cam_qc(2:end,1),cam_qc(2:end,2),'or');
        plot(cam_qc(1,1),cam_qc(1,2),'xr');
        %plot(cam_sfmPos(:,1),cam_sfmPos(:,2),'.-','Color','#004000');
        plot(cam_estTrj(:,1),cam_estTrj(:,2)       ,'.-','Color','#80E080','LineWidth',7,'MarkerEdgeColor','#80C080','MarkerSize',7);
        plot(cam_trajectory(:,1),cam_trajectory(:,2),':','Color','#004000','LineWidth',1);
        axis ij;
        axis equal;
        axis(camFrame);
    end
    if(1)
        subplot(2,2,1);
        hold off;    
        plot(camPrev_frames(:,1),camPrev_frames(:,2),'.-','Color','#F0C000');
        hold on;
        plot(camPrev_line1(:,1),camPrev_line1(:,2),'.-','Color','#C0C0C0','LineWidth',2,'MarkerEdgeColor','#808080','MarkerSize',5);
        plot(camPrev_line2(:,1),camPrev_line2(:,2),'.-','Color','#FFC0C0','LineWidth',2,'MarkerEdgeColor','#C08080','MarkerSize',5);
        plot(camPrev_roadmark(:,1),camPrev_roadmark(:,2),'x-b');
        plot(camPrev_qc(2:end,1),camPrev_qc(2:end,2),'or');
        plot(camPrev_qc(1,1),camPrev_qc(1,2),'xr');
        plot(camPrev_estTrj(:,1),camPrev_estTrj(:,2)       ,'.-','Color','#80E080','LineWidth',7,'MarkerEdgeColor','#80C080','MarkerSize',7);
        plot(camPrev_trajectory(:,1),camPrev_trajectory(:,2),':','Color','#004000','LineWidth',1);
        %plot(camPrev_sfmPos(:,1),camPrev_sfmPos(:,2),'.-','Color','#004000');
        axis ij;
        axis equal;
        axis(camFrame);
    end
   
    if(1)
        subplot(2,1,2);
        hold off;
        plot3(frames(:,1),frames(:,2),frames(:,3),'.-','Color','#F0C000');
        hold on;        
        plot3(line1(:,1),line1(:,2),line1(:,3),'-','Color','#C0C0C0');
        plot3(line2(:,1),line2(:,2),line2(:,3),'-','Color','#FFC0C0');
        plot3(pyramid(:,1),pyramid(:,2),pyramid(:,3),'.-','Color','#FF0000');
        plot3(roadmark(:,1),roadmark(:,2),roadmark(:,3),'x-b');
        %plot3(sfmPos(1:k,1),sfmPos(1:k,2),sfmPos(1:k,3),'.-','Color','#004000');
        plot3(estTrj(1:k,1),estTrj(1:k,2),estTrj(1:k,3)     ,'.-','Color','#80E080','LineWidth',3,'MarkerEdgeColor','#40C040','MarkerSize',3);
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),':','Color','#004000','LineWidth',1);
        
        axis equal;
        if(1)
            viewAngle=-rad2deg(camPos(5))+30;
            view(viewAngle,10);
            %view(viewAngle,90);
            viewRangeX=10+30*abs(sin(deg2rad(viewAngle)));
            viewRangeY=10+30*abs(cos(deg2rad(viewAngle)));
            viewRangeZt= 3;
            viewRangeZb=10;
            axis([-viewRangeX+camPos(1), ...
                  +viewRangeX+camPos(1), ...
                  -viewRangeY+camPos(2),...
                  +viewRangeY+camPos(2),...
                  -viewRangeZb+camPos(3),...
                  +viewRangeZt+camPos(3)    ]);
        else            
            view(0,90);            
            axis equal;
        end
        
    end
    
    drawnow;


    %% Pudate Prev
    camPrevPos=camPos;

    %drawnow limitrate;
    if(video_enable )
       frame = getframe(gcf);
       writeVideo(video_handler,frame);
    end

end

%% Video Terminator
if(video_enable)
    close(video_handler);
end

%% -------------------------------------------------------------------
%% -------------------------------------------------------------------
%% -- Shape System
function [shape]=getShapeConcatinate(shape1,shape2)
    shape12=shape1(end-1,1:2);
    shape13=shape1(end  ,1:2);    
    shape1d=shape13-shape12;
    th1    =atan2(shape1d(1,2),shape1d(1,1));

    shape20=shape2(1,1:2);
    shape21=shape2(2,1:2);
    shape2d=shape21-shape20;
    th2    =atan2(shape2d(1,2),shape2d(1,1));
    C = cos(th1-th2);
    S = sin(th1-th2);
    R =[C,S;-S,C];
    shape2C=(shape2-shape20)*R+shape13;
    shape=[shape1;shape2C(2:end,:)];
end

function [shape]=getShapeStraight(x_length,y_length)
    shape=[       0,       0 ;
           x_length,y_length];
end
function [shape]=getShapeCurve(R,angle)
    Rabs=abs(R);
    length_step=1;
    angle_step1=2*pi/72;
    angle_step2=length_step/Rabs;
    angle_step = min(angle_step1,angle_step2);
    angle_range=(0:angle_step:angle)';
    shape=[Rabs*sin(angle_range),Rabs-R*cos(angle_range)];
end

%% -- Corrdinate system

function [param]=getPlaneParam(p1,p2,p3)
    v1=p1-p2;
    v2=p3-p2;
    op=cross(v1,v2);
    d=op*(-p2)';
    param=[op,d];
end

function [p]=getPlaneProjection(P2,camRot,camOfs,param)
    
    p_size=size(P2,1);
    p1    =camOfs;
    p2mp1 =P2/camRot;    
    pden  =param(1,1:3)*p2mp1';
    pnom  =param(1,4)+param(1,1:3)*p1';
    p     =zeros(p_size,3);
    for i=1:p_size
        p(i,1:3)=-pnom/pden(1,i)*p2mp1(i,1:3)+p1;
    end      
end


function [q1,q2,p,valid]=getAnotatedPoint(line1, line2, camF,camFrame, camPos1,camPos2,pointCnt)
    cnt=round(pointCnt/2)*2;
    %cnt=20;
    cnth=floor(cnt/2);
    cnt_idx=1;
    q1=zeros(cnt,2);
    q2=zeros(cnt,2);
    p =zeros(cnt,3);
    valid=0;

    ofs1=camPos1(1:3);
    rot1=getRotMatrix(camPos1(4:end));
    ofs2=camPos2(1:3);
    rot2=getRotMatrix(camPos2(4:end));
    cline11=getTrans(line1,rot1,ofs1);
    cline12=getTrans(line2,rot1,ofs1);
    cline21=getTrans(line1,rot2,ofs2);
    cline22=getTrans(line2,rot2,ofs2);
    
    %%
    cline1_size=size(line1,1);
    cam11=zeros(cline1_size,2);
    cam21=zeros(cline1_size,2);
    valid_cnt1=0;
    for i=1:cline1_size
        if(0<cline11(i,3) && 0<cline21(i,3) && ...
           camFrame(4)*cline11(i,3)>camF*cline11(i,2) && ... 
           camFrame(4)*cline21(i,3)>camF*cline21(i,2) &&...
           camFrame(1)*cline11(i,3)<camF*cline12(i,1) &&... 
           camFrame(1)*cline21(i,3)<camF*cline22(i,1) &&...
           camFrame(2)*cline11(i,3)>camF*cline12(i,1) &&... 
           camFrame(2)*cline21(i,3)>camF*cline22(i,1) )            
            cam11(i,1)=+camF*cline11(i,1)/cline11(i,3);
            cam11(i,2)=+camF*cline11(i,2)/cline11(i,3);            
            cam21(i,1)=+camF*cline21(i,1)/cline21(i,3);
            cam21(i,2)=+camF*cline21(i,2)/cline21(i,3);
            valid_cnt1=valid_cnt1+1;
        else            
            cam11(i,1)=0;
            cam11(i,2)=camFrame(3);
            cam21(i,1)=0;
            cam21(i,2)=camFrame(3);
        end
    end
    cnt_idx_end=cnt_idx+min( (valid_cnt1-1), floor(cnth-1) );
    if(valid_cnt1>0)
        %[~,cam11_idx]=sort(cam11(:,2),"ascend");
        [~,cam11_idx]=sort(cam11(:,2),"descend");
        order_idx = 1;
        for i=cnt_idx:cnt_idx_end
            p(i,1:3) =line1(cam11_idx(order_idx),1:3);        
            q1(i,1:2)=cam11(cam11_idx(order_idx),1:2);        
            q2(i,1:2)=cam21(cam11_idx(order_idx),1:2);  
            order_idx = order_idx + 1;      
        end
        cnt_idx=cnt_idx_end+1;
    end
    %%
    cline2_size=size(line2,1);
    cam12=zeros(cline2_size,2);
    cam22=zeros(cline2_size,2);
    valid_cnt2=0;
    for i=1:cline2_size
        if(0<cline12(i,3) && 0<cline22(i,3) && ...
           camFrame(4)*cline12(i,3)>camF*cline12(i,2) && ... 
           camFrame(4)*cline22(i,3)>camF*cline22(i,2) &&...
           camFrame(1)*cline12(i,3)<camF*cline12(i,1) &&... 
           camFrame(1)*cline22(i,3)<camF*cline22(i,1) &&...
           camFrame(2)*cline12(i,3)>camF*cline12(i,1) &&... 
           camFrame(2)*cline22(i,3)>camF*cline22(i,1) )
            cam12(i,1)=+camF*cline12(i,1)/cline12(i,3);
            cam12(i,2)=+camF*cline12(i,2)/cline12(i,3);
            cam22(i,1)=+camF*cline22(i,1)/cline22(i,3);
            cam22(i,2)=+camF*cline22(i,2)/cline22(i,3);
            valid_cnt2=valid_cnt2+1;
        else            
            cam12(i,1)=0;
            cam12(i,2)=camFrame(3);
            cam22(i,1)=0;
            cam22(i,2)=camFrame(3);
        end
    end

    cnt_idx_end=min( cnt_idx+(valid_cnt2-1), cnt );
    if(valid_cnt2>0)
        %[~,cam12_idx]=sort(cam12(:,2),"ascend");
        [~,cam12_idx]=sort(cam12(:,2),"descend");
        order_idx = 1;
        for i=cnt_idx:cnt_idx_end
            p(i,1:3) =line2(cam12_idx(order_idx),1:3);        
            q1(i,1:2)=cam12(cam12_idx(order_idx),1:2);        
            q2(i,1:2)=cam22(cam12_idx(order_idx),1:2);        
            order_idx=order_idx+1;
        end
    end

    if( cnt_idx_end >= cnt  )
        valid = 1;
    end
    refidx=zeros(1,cnt);
    for i=1:cnth
        refidx(i*2-1)=i;
        refidx(i*2  )=i+cnth;
    end
        
    p = p(refidx,:);
    q1=q1(refidx,:);
    q2=q2(refidx,:);

end

function [P]=getTrans(p,rot,ofs)
    ofss=repmat(ofs,[size(p,1),1]);
    P=(p-ofss)*rot;
end

function [rot]=getRotMatrix(pos)
    pos_size  =size(pos,2);
    if( pos_size>=1)
        cos_pitch=cos(pos(1));
        sin_pitch=sin(pos(1));
    else
        cos_pitch=1.0;
        sin_pitch=0.0;
    end
    Rpitch=[1,         0,         0;
            0,+cos_pitch,+sin_pitch;
            0,-sin_pitch,+cos_pitch];
            
    if( pos_size>=2)
        cos_yaw  =cos(pos(2));
        sin_yaw  =sin(pos(2));
    else
        cos_yaw  =1.0;
        sin_yaw  =0.0;
    end
    Ryaw=[+cos_yaw,0,+sin_yaw;
                 0,1,       0;
          -sin_yaw,0,+cos_yaw];                 

    if( pos_size>=3)
        cos_roll  =cos(pos(3));
        sin_roll  =sin(pos(3));
    else
        cos_roll  =1.0;
        sin_roll  =0.0;
    end
    Rroll=[1,        0,        0;
           0,+cos_roll,+sin_roll;
           0,-sin_roll,+cos_roll]';
     
    rot=Rroll*Ryaw*Rpitch;   
end

function [q0]=getVanishmentPoint(q) 
    b=(q(2,1:2)-q(1,1:2))';
    A=[(q(3,1:2)-q(1,1:2))', -(q(4,1:2)-q(2,1:2))'];
    a=A\b;
    q0=a(1)*(q(3,1:2)-q(1,1:2))+q(1,1:2);
end

function [R,theta,phi,psi]=getRotarionMatrixFromVanishmentPoint(q0,f)
    theta =atan2(q0(1,1),f);%Yaw
    phi   =atan2(q0(1,2),f);%Pitch
    psi   =0;               %Roll
    Cphi  =cos(phi);
    Sphi  =sin(phi);
    Ctheta=cos(theta);
    Stheta=sin(theta);
    Cpsi  =cos(psi);
    Spsi  =sin(psi);
    Rtheta=[+Ctheta,+Stheta,0;
            -Stheta,+Ctheta,0;
                  0,      0,1];
    Rphi  =[+Cphi,0,-Sphi;
                0,1,    0;
            +Sphi,0,+Cphi];
    Rpsi  =[1,    0,    0;
            0,+Cpsi,+Spsi;
            0,-Spsi,+Cpsi];
    
    R = Rtheta * Rphi * Rpsi;

end

function [R,yaw,pitch,roll]=getRotarionMatrixFromVanishmentPointR(q0,f)
    yaw   =atan2(q0(1,1),f); %Yaw
    pitch =atan2(q0(1,2),f); %Pitch
    roll  =0;                %Roll

    theta = -pitch ;
    phi   = -yaw+pi/2 ;
    psi   = -roll - pi/2 ;

    Ctheta=cos(theta);
    Stheta=sin(theta);
    Cphi  =cos(phi);
    Sphi  =sin(phi);
    Cpsi  =cos(psi);
    Spsi  =sin(psi);
%     Rtheta=[+Ctheta,+Stheta,0;
%             -Stheta,+Ctheta,0;
%                   0,      0,1];
    Rtheta  =[1,    0,    0;
              0,+Ctheta,+Stheta;
              0,-Stheta,+Ctheta];
    Rphi  =[+Cphi,0,-Sphi;
                0,1,    0;
            +Sphi,0,+Cphi];
    Rpsi  =[1,    0,    0;
            0,+Cpsi,+Spsi;
            0,-Spsi,+Cpsi];
    
    %R = Rtheta * Rphi * Rpsi;
    R = Rpsi * Rphi * Rtheta ;

end



function [c2]=getNextPos(q1,q2,q10,q20,c1,f,ref)
    
    %% Derive rotation matrix from vanishment point                 
    [R1,theta1,phi1,psi1]=getRotarionMatrixFromVanishmentPoint(q10,f);
    [R2,theta2,phi2,psi2]=getRotarionMatrixFromVanishmentPoint(q20,f);
    deltaAngle=[theta2-theta1,phi2-phi1,psi2-psi1];

    %% 
    q1_size=size(q1,1);
    q2_size=size(q2,1);
    PS1=[repmat(f,[q1_size,1]),-q1(:,1),+q1(:,2)];
    PS2=[repmat(f,[q2_size,1]),-q2(:,1),+q2(:,2)];
    PS1pR1=PS1/R1;
    PS2pR2=PS2/R2;
    switch(2)
        case 0
            A=[        eye(3),         eye(3),         eye(3);
               +PS2pR2(1,1:3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3), +PS2pR2(2,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), +PS2pR2(3,1:3);
               -PS1pR1(1,1:3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3), -PS1pR1(2,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), -PS1pR1(3,1:3)];
            b=[c1(1,1:3),c1(1,1:3),c1(1,1:3)];
            x=b/A;
        case 1
            A=[        eye(3),         eye(3),         eye(3);
               +PS2pR2(1,1:3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3), +PS2pR2(2,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), +PS2pR2(3,1:3);
                   zeros(1,3), -PS1pR1(2,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), -PS1pR1(3,1:3)];
            %TS11=norm(ref(1,1:3)-c1)/norm(PS1pR1(1,1:3));
            %b=[c1+TS11*PS1pR1(1,1:3),c1,c1];
            b=[ref(1,1:3),c1(1,1:3),c1(1,1:3)];
            x=b/A;
        case 2
            A=[        eye(3),         eye(3),         eye(3),         eye(3);
               +PS2pR2(1,1:3),     zeros(1,3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3), +PS2pR2(2,1:3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), +PS2pR2(3,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3),     zeros(1,3), +PS2pR2(4,1:3);
                   zeros(1,3), -PS1pR1(2,1:3),     zeros(1,3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3), -PS1pR1(3,1:3),     zeros(1,3);
                   zeros(1,3),     zeros(1,3),     zeros(1,3), -PS1pR1(4,1:3)];
            %TS11=norm(ref(1,1:3)-c1)/norm(PS1pR1(1,1:3));
            %b=[c1+TS11*PS1pR1(1,1:3),c1,c1];
            b=[ref(1,1:3),c1(1,1:3),c1(1,1:3),c1(1,1:3)];
            x=b/A;
        case 3
            PSPS=zeros(q1_size,9);
            for k=1:q1_size
                for j=1:3
                    for i=1:3
                        l=(j-1)*3+(i-1)+1;
                        PSPS(k,l)=PS2(k,i)*PS2(k,j);
                    end
                end
            end
            %[U,D,V]=svd(PSPS);
            %e=V(:,9);
            %E=sart(2)*()
            x=c1(1,1:3);
    end
    
    c2=[x(1,1:3),c1(1,4:6)+deltaAngle];

end
