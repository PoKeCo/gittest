trj_th=deg2rad((0:-1:-45)');
trj_R=100;
trj_pos=[trj_R*sin(trj_th),trj_R-trj_R*cos(trj_th),trj_th];


world_w =2.75/2;
world_h =0;
world_d0=5;
world_d1=10;

world_paint=[+world_d0,-world_w,world_h;
             +world_d0,+world_w,world_h;
             +world_d1,+world_w,world_h;
             +world_d1,-world_w,world_h;
             +world_d0,-world_w,world_h];
%world_paint=[(rand(100,1)*2-1)*world_d1,(rand(100,1)*2-1)*world_w,zeros(100,1)];

plane_param=getPlaneParam(world_paint(1,1:3),...
                          world_paint(2,1:3),...
                          world_paint(3,1:3));


camF  =1;
camFrame=[-1,+1,-9/16,+9/16];
init=1;
path1=[];
path2=[];
k=1;
for t=10:-0.5:-10
    %th=deg2rad(t);
    %camPos=[cos(th)*7.5,sin(th)*7.5,2.2,deg2rad(0),deg2rad(t+180),deg2rad(0)];    
    %camPos=[t,0,1.2,deg2rad(0),deg2rad(0),deg2rad(0)];    
    th1=5*sin(deg2rad(t/6*360));
    th2=5*sin(deg2rad(t/3*360));
    th3=5*sin(deg2rad(t/2*360));
    %camPos=[t,0,1.2,deg2rad(th1),deg2rad(th2),deg2rad(th3)];    
    %camPos=[t,0,1.2,deg2rad(0),deg2rad(0),deg2rad(0)];    
    %camPos=[100*sin(t/10*pi/4),100+100*cos(t/10*pi/4),1.2,deg2rad(0),deg2rad(0),deg2rad(0)];    
    %camPos=[t,0,1.2,deg2rad(0),deg2rad(0),deg2rad(th1)];
    camPos=[trj_pos(k,1),trj_pos(k,2),1.2,deg2rad(0),trj_pos(k,3),deg2rad(0)];
    k=k+1;
    [cam,camRot,camOfs]=world2cam(world_paint,camF,camPos);
    
    if(1)
        t_xyzeze=zeros(3,3);
        xeyeze  =zeros(3,3);
        cam_param=zeros(1,4);
        cam_param(1,4)  =plane_param(1,4)+plane_param(1,1:3)*camOfs';
        cam_param(1,1:3)=plane_param(1,1:3)*inv(camRot)';
        uevef=[camF,-camFrame(1),camFrame(3);
               camF,           0,camFrame(3);
               camF,-camFrame(2),camFrame(3)];
        for i=1:3
            t_xeyeze(i,1:3)=-cam_param(4)/(cam_param(1:3)*uevef(i,1:3)')*uevef(i,1:3);
            %xeyeze(i,1:3)=t_xeyeze(i,1:3)*inv(rot)+ofs;
            xeyeze(i,1:3)=t_xeyeze(i,1:3)/camRot+camOfs;
        end 
        init=1;        
        path1=[path1;xeyeze];
    end
    if(1)
        P2=[camF,-camFrame(1),camFrame(3);            
            camF,-camFrame(2),camFrame(3)];
        xeyeze1=getPlaneProjection(P2,camRot,camOfs,plane_param );
        
        wall_p1=xeyeze1(1,1:3);
        wall_p2=xeyeze1(2,1:3);
        wall_p3=[xeyeze1(2,1:2),xeyeze1(2,3)+1];

        P2=[camF,-camFrame(2),camFrame(4);            
            camF,-camFrame(1),camFrame(4);
            camF,-camFrame(1),camFrame(3)];
        wall_param=getPlaneParam(wall_p1,...
                                 wall_p2,...
                                 wall_p3);
        xeyeze2=getPlaneProjection(P2,camRot,camOfs,wall_param );
                                 
        path2=[path2;xeyeze1;xeyeze2];
    end

    %cam_xeyeze = world2cam(xeyeze,camF,camPos);
    cam_xeyeze1 = world2cam(path1,camF,camPos);    
    cam_xeyeze2 = world2cam(path2,camF,camPos);    
    %cam_xeyeze = world2cam(xeyeze,camF,[0,0,0,0,0,0]);
    %cam_xeyeze = world2cam(world_paint,camF,camPos);
    %cam_xeyeze1(:,1),cam_xeyeze1(:,2),'s',...

    figure(1);
    subplot(2,1,1);
    plot(cam_xeyeze1(:,1),cam_xeyeze1(:,2),'sr',...
         cam_xeyeze2(:,1),cam_xeyeze2(:,2),'.-y',...
         cam(:,1),cam(:,2),'x-b'         );
    axis equal;
    axis(camFrame);    
    %pause(1);
    %figure(2);
    subplot(2,1,2);
    plot3(path2(:,1),path2(:,2),path2(:,3));
    axis equal;
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