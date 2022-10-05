%% ------------------------------------------------------------------------
addpath('../../pllib');

video_enable=0;
if(video_enable)
    video_handler=VideoWriter('testAssginIdealX.avi','Motion JPEG AVI');
    %video_handler=VideoWriter('testTraceModel5.avi','Motion JPEG 2000');
    %video_handler.FrameRate=15*4;
    video_handler.FrameRate=10;
    open(video_handler);
end

figure(1);
clf;

Fs=10;
Ts=1/Fs;

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
else
shape1=getShapeStraight(-shapeStraightLeng,0);
shape2=getShapeCurve(-shapeR,deg2rad(45));
shape3=getShapeCurve( shapeR/2,deg2rad(60));
shape4=getShapeCurve(-shapeR/2,deg2rad(60));
shape =getShapeConcatinate(shape1,shape2);
shape =getShapeConcatinate(shape,shape3);
shape =getShapeConcatinate(shape,shape4);
shape =getShapeConcatinate(shape,shape1);

end

trajectoryXY=getChopped(shape,deltaStep);
trajectoryXYYaw=appendYaw(trajectoryXY);
trajectorySize=size(trajectoryXY,1);
trajectoryZ = zeros(trajectorySize,1);
trajectory=[trajectoryXY(:,1:2),trajectoryZ,trajectoryXYYaw(:,3)+pi];

sfmPos=zeros(trajectorySize,6);
estTrj=zeros(trajectorySize,6);

lineSize=trajectorySize;
lineOfs=2.75/2*ones(lineSize,1);
lineZ  =zeros(lineSize,1);
line1XY=getOfsPath(trajectoryXY,-lineOfs);
line2XY=getOfsPath(trajectoryXY,+lineOfs);
line1=[line1XY,lineZ];
line2=[line2XY,lineZ];

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

u_fov=1;
a_aspect=9/16;
FOV_H=2*pi*120/360;
v_fov=a_aspect*u_fov;
camF=u_fov/tan(FOV_H/2);
camFrame=[-u_fov,+u_fov,-v_fov,v_fov];
%camF  =1;
%camFrame=[-1,+1,-9/16,+9/16];

init=1;
t=0;
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
    YawOfs   =5;
    PitchOfs =0;
    th1=deg2rad(PitchAmp*sin(2*pi*PitchFreq*t)+PitchOfs);
    th2=deg2rad(YawAmp*sin(2*pi*YawFreq*t)+YawOfs);
    t=t+Ts;

    camPos=[trajectory(k,1:2),camZ,...
           deg2rad(0)+th1,trajectory(k,4)+th2,deg2rad(0)];
    
    [cam_roadmark,camRot,camOfs]=world2cam(roadmark,camF,camPos);
    
    %% Create Frame Wall 
    P2=[camF,           0,camFrame(3);            
        camF,-camFrame(2),camFrame(3)];
    frame1=getPlaneProjection(P2,camRot,camOfs,paint_param );
    
    wall_p1=frame1(1,1:3);
    wall_p2=frame1(2,1:3);
    wall_p3=[frame1(2,1:2),frame1(2,3)+1];

    P2=[camF,-camFrame(2),camFrame(4);            
        camF,-camFrame(1),camFrame(4);            
        camF,-camFrame(1),camFrame(3);
        camF,           0,camFrame(3)];
    wall_param=getPlaneParam(wall_p1,wall_p2,wall_p3);

    frame2=getPlaneProjection(P2,camRot,camOfs,wall_param );
    append_frame=[frame1;frame2];                                 
    frames=[frames;append_frame]; 
    
    %% Append Frame
    append_frame_size=size(append_frame,1);
    pyramid = reshape([append_frame,[append_frame(2:end,:);append_frame(1,:)],repmat(camPos(1,1:3),[append_frame_size,1])]',[3,append_frame_size*3])';
    
    %% Extruct Annotation point
    [cam_q,camPrev_q,refPos,isValid]=getAnotatedPoint(line1, line2, camF,camFrame, camPos,camPrevPos);
    if(isValid)    
        %% Derive Vanishment Point
        cam_q0    =getVanishmentPoint(cam_q);
        camPrev_q0=getVanishmentPoint(camPrev_q);
        cam_qc    =[cam_q0    ;cam_q    ];
        camPrev_qc=[camPrev_q0;camPrev_q];
        c1        = sfmPos(k-1,:);
        %% Derive rotation matrix from vanishment point                 
        %[R1,theta,phi,psi]=getRotarionMatrixFromVanishmentPoint(camPrev_q0,camF);
        %[camPrevPos(4),phi]        
        %[R1,theta,phi,psi]=getRotarionMatrixFromVanishmentPoint(cam_q0    ,camF);
        c2=getNextPos(camPrev_q ,cam_q,...
                      camPrev_q0,cam_q0,...
                      c1,camF,refPos);
        sfmPos(k,:)=c2;
        %sfmPos(k,:)=camPos(1,:);
                
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
    cam_frames     = world2cam(frames,camF,camPos); 
    cam_trajectory = world2cam(trajectory(:,1:3),camF,camPos); 
    cam_line1      = world2cam(line1,camF,camPos); 
    cam_line2      = world2cam(line2,camF,camPos); 
    cam_estTrj     = world2cam(estTrj(1:k,1:3),camF,camPos);
    %cam_sfmPos     = world2cam(sfmPos(1:k,1:3),camF,camPos); 

    %% Create Cam Frames
    camPrev_roadmark   = world2cam(roadmark,camF,camPrevPos);
    camPrev_frames     = world2cam(frames,camF,camPrevPos); 
    camPrev_trajectory = world2cam(trajectory(:,1:3),camF,camPrevPos); 
    camPrev_line1      = world2cam(line1,camF,camPrevPos); 
    camPrev_line2      = world2cam(line2,camF,camPrevPos);
    camPrev_estTrj     = world2cam(estTrj(1:k,1:3),camF,camPrevPos);
    %camPrev_sfmPos     = world2cam(sfmPos(1:k,1:3),camF,camPrevPos); 
    

    %% Pudate Prev
    camPrevPos=camPos;

    %% 
    %drawnow nocallbacks;
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
        plot3(estTrj(1:k,1),estTrj(1:k,2),estTrj(1:k,3)     ,'.-','Color','#80E080','LineWidth',3,'MarkerEdgeColor','#80C080','MarkerSize',3);
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),':','Color','#004000','LineWidth',1);
        
        axis equal;
        if(1)
            viewAngle=rad2deg(camPos(5))-20;
            view(viewAngle,20);
            viewRangeX=5+40*abs(cos(deg2rad(viewAngle)));
            viewRangeY=5+40*abs(sin(deg2rad(viewAngle)));
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


function [q1,q2,p,valid]=getAnotatedPoint(line1, line2, camF,camFrame, camPos1,camPos2)

    cnt=10;
    cnth=cnt/2;
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
        if(0<cline11(i,1) && 0<cline21(i,1) && ...
           camFrame(3)*cline11(i,1)<camF*cline11(i,3) && ... 
           camFrame(3)*cline21(i,1)<camF*cline21(i,3) )            
            cam11(i,1)=-camF*cline11(i,2)/cline11(i,1);
            cam11(i,2)=+camF*cline11(i,3)/cline11(i,1);            
            cam21(i,1)=-camF*cline21(i,2)/cline21(i,1);
            cam21(i,2)=+camF*cline21(i,3)/cline21(i,1);
            valid_cnt1=valid_cnt1+1;
        else            
            cam11(i,1)=0;
            cam11(i,2)=camFrame(4);
            cam21(i,1)=0;
            cam21(i,2)=camFrame(4);
        end
    end
    cnt_idx=1;
    cnt_idx_end=cnt_idx+min(valid_cnt1,cnth)-1;
    ref_idx=1;
    if( valid_cnt1>=2)
        [~,cam11_idx]=sort(cam11(:,2),"ascend");
        for i=cnt_idx:cnt_idx_end
            p(i,1:3) =line1(cam11_idx(ref_idx),1:3);
            q1(i,1:2)=cam11(cam11_idx(ref_idx),1:2);
            q2(i,1:2)=cam21(cam11_idx(ref_idx),1:2);
            ref_idx=ref_idx+1;
        end
    end
    cnt_idx=cnt_idx_end+1;

    %%
    cline2_size=size(line2,1);
    cam12=zeros(cline2_size,2);
    cam22=zeros(cline2_size,2);
    valid_cnt2=0;
    for i=1:cline2_size
        if(0<cline12(i,1) && 0<cline22(i,1) && ...
           camFrame(3)*cline12(i,1)<camF*cline12(i,3) && ... 
           camFrame(3)*cline22(i,1)<camF*cline22(i,3))            
            cam12(i,1)=-camF*cline12(i,2)/cline12(i,1);
            cam12(i,2)=+camF*cline12(i,3)/cline12(i,1);
            cam22(i,1)=-camF*cline22(i,2)/cline22(i,1);
            cam22(i,2)=+camF*cline22(i,3)/cline22(i,1);
            valid_cnt2=valid_cnt2+1;
            else            
            cam12(i,1)=0;
            cam12(i,2)=camFrame(4);
            cam22(i,1)=0;
            cam22(i,2)=camFrame(4);
        end
    end

    cnt_idx_end=cnt_idx+min(valid_cnt1,cnth)-1;
    if( valid_cnt2>=2)
        [~,cam12_idx]=sort(cam12(:,2),"ascend");
        ref_idx=1;
        for i=cnt_idx:cnt_idx_end
            p(i,1:3) =line2(cam12_idx(ref_idx),1:3);
            q1(i,1:2)=cam12(cam12_idx(ref_idx),1:2);
            q2(i,1:2)=cam22(cam12_idx(ref_idx),1:2);
            ref_idx=ref_idx+1;
        end
    end

    valid_cnt=valid_cnt1+valid_cnt2;
    if( valid_cnt1>=2 && valid_cnt2>=2 && valid_cnt>=cnt)
        valid = 1;
        zigzag=zeros(cnt,1);
        for i=1:cnth
            zigzag((i-1)*2+1 )=i;
            zigzag((i-1)*2+2 )=i+cnth;
        end
        p =p(zigzag,:);
        q1=q1(zigzag,:);
        q2=q2(zigzag,:);
    end

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
    Rpitch=[+cos_pitch,0,+sin_pitch;
                     0,1,         0;
            -sin_pitch,0,+cos_pitch]';

    if( pos_size>=2)
        cos_yaw  =cos(pos(2));
        sin_yaw  =sin(pos(2));
    else
        cos_yaw  =1.0;
        sin_yaw  =0.0;
    end
    Ryaw=[+cos_yaw,+sin_yaw,0;
          -sin_yaw,+cos_yaw,0;
                 0,       0,1]';          

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
    rot=Ryaw*Rpitch*Rroll;    
end

function [q0]=getVanishmentPoint(q) 
    b=(q(2,1:2)-q(1,1:2))';
    A=[(q(3,1:2)-q(1,1:2))', (q(4,1:2)-q(2,1:2))'];
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

function [c2]=getNextPos(q1,q2,q10,q20,c1,f,ref)
    
    %% Derive rotation matrix from vanishment point                 
    [R1,theta1,phi1,psi1]=getRotarionMatrixFromVanishmentPoint(q10,f);
    [R2,theta2,phi2,psi2]=getRotarionMatrixFromVanishmentPoint(q20,f);
    deltaAngle=[theta2-theta1,phi2-phi1,psi2-psi1];
    R=R1\R2;

    %% 
    q1_size=size(q1,1);
    q2_size=size(q2,1);
    PS1=[repmat(f,[q1_size,1]),-q1(:,1),+q1(:,2)];
    PS2=[repmat(f,[q2_size,1]),-q2(:,1),+q2(:,2)];
    PS1pR1=PS1/R1;
    PS2pR2=PS2/R2;
    switch(3)
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
            c2=[x(1,1:3),c1(1,4:6)+deltaAngle];
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
            c2=[x(1,1:3),c1(1,4:6)+deltaAngle];
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
            c2=[x(1,1:3),c1(1,4:6)+deltaAngle];
        case 3
            B=zeros(q1_size,3*3);
            for k=1:q1_size
                for j=1:3
                    for i=1:3
                        r=(j-1)*3+i;
                        B(k,r)=PS2(k,j)*PS1(k,i);
                    end
                end
            end
            [U, D, V]=svd(B);
            e=V(:,9);
            E=sqrt(2)*[e(1:3)';e(4:6)';e(7:9)'];
            [U D V]=svd(E); 
            D=diag([(D(1,1)+D(2,2))/2, (D(1,1)+D(2,2))/2, 0]);
            hatE=U*D*V';
            [U, D, V]=svd(hatE);
            t=U(:,3);
            Rz=[0 1 0;-1 0 0;0 0 1];
            R1=U*Rz*V';
            R2=U*Rz'*V';
            
            c2=[c1(1,1:3)+t(1:3,1)',c1(1,4:6)+deltaAngle];
    end

end
