addpath('../../pllib');

figure(1);
clf;

Fs=10;
Ts=1/Fs;

deltaStep=60/3.6/Fs;

shapeTh=deg2rad((0:-1:-30)');
shapeR=100;
shapeStraightLeng=50;
shapeCurve=[shapeR*sin(shapeTh),shapeR-shapeR*cos(shapeTh)];
shapeStraight1=[shapeStraightLeng,0];
shapeStraight2=(shapeCurve(end,:)-shapeCurve(end-1,:))/deltaStep*shapeStraightLeng+shapeCurve(end,:);
shape=[shapeStraight1;shapeCurve;shapeStraight2];


trajectoryXY=getChopped(shape,deltaStep);
trajectoryXYYaw=appendYaw(trajectoryXY);
trajectorySize=size(trajectoryXY,1);
trajectoryZ = zeros(trajectorySize,1);
trajectory=[trajectoryXY(:,1:2),trajectoryZ,trajectoryXYYaw(:,3)+pi];

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


camF  =1;
camFrame=[-1,+1,-9/16,+9/16];
init=1;
t=0;
append_frame_size=6;
frames=[];
%frames=zeros(append_frame_size*trajectorySize,3 );
for k=1:trajectorySize

    Freq=0.5;
    th1=deg2rad(5*sin(2*pi*Freq*t));
    t=t+Ts;

    camPos=[trajectory(k,1:2),camZ,...
           deg2rad(0),trajectory(k,4),deg2rad(0)];
    
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
    %append_idx_begin=append_frame_size*(k-1)+1;
    %append_idx_end  =append_frame_size*k;
    %frames(append_idx_begin:append_idx_end,1:3)=append_frame;
    
    %% Append Frame
    append_frame_size=size(append_frame,1);
    pyramid = reshape([append_frame,[append_frame(2:end,:);append_frame(1,:)],repmat(camPos(1,1:3),[append_frame_size,1])]',[3,append_frame_size*3])';

    %% Create Cam Frames
    cam_frames     = world2cam(frames,camF,camPos); 
    cam_trajectory = world2cam(trajectory(:,1:3),camF,camPos); 
    cam_line1      = world2cam(line1,camF,camPos); 
    cam_line2      = world2cam(line2,camF,camPos); 

    %% 
    %drawnow nocallbacks;
    if(1)
        subplot(2,1,1);
        hold off;    
        plot(cam_trajectory(:,1),cam_trajectory(:,2),'-','Color','#C0FFC0','LineWidth',2);
        hold on;
        plot(cam_line1(:,1),cam_line1(:,2),'-','Color','#C0C0C0','LineWidth',2);
        plot(cam_line2(:,1),cam_line2(:,2),'-','Color','#FFC0C0','LineWidth',2);
        plot(cam_frames(:,1),cam_frames(:,2),'.-','Color','#F0C000');
        plot(cam_roadmark(:,1),cam_roadmark(:,2),'x-b');        
        axis equal;
        axis(camFrame);
    end
   
    if(1)
        subplot(2,1,2);
        hold off;
        %view(0,90);    
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),'-','Color','#C0FFC0','LineWidth',2);
        hold on;
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),'-','Color','#C0FFC0','LineWidth',2);
        plot3(line1(:,1),line1(:,2),line1(:,3),'-','Color','#C0C0C0');
        plot3(line2(:,1),line2(:,2),line2(:,3),'-','Color','#FFC0C0');
        plot3(pyramid(:,1),pyramid(:,2),pyramid(:,3),'.-','Color','#FF0000');
        plot3(frames(:,1),frames(:,2),frames(:,3),'.-','Color','#F0C000');
        plot3(roadmark(:,1),roadmark(:,2),roadmark(:,3),'x-b');
        axis equal;
    end

    drawnow;

end

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