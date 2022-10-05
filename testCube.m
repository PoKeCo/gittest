

%clf
% cubeNode0=[+0,+1,-1;
%           +0,-1,-1;
%           +0, 0, 0;
%           +0,-1,+1;
%           +0,+1,+1;
%           +2,+1,-1;
%           +2,-1,-1;
%           +2, 0, 0;
%           +2,-1,+1;
%           +2,+1,+1];
cubeNode=[+0,+1,-1;
          +0,+0,-1;
          +0,-1,-1;
          +1,-1,-1;
          +2,-1,-1;          
          +2,-1, 0;
          +1,-1, 0;
          +0,-1, 0;
          +0,+0, 0;
          +0,+1, 0;
          +0,+1,+1;
          +0,+0,+1;
          +0,-1,+1;
          +1,-1,+1;
          +2,-1,+1;
          +2,+1,+1];
          
cubeShapeIdx=[(1:size(cubeNode,1))';
               11;10;1;2;9;12;13;8;3;4;7;14;15;6];
cubeShape=cubeNode(cubeShapeIdx,:);

%% Show 3d shape
subplot(2,1,2);
plot3(cubeNode (:,1),cubeNode (:,2),cubeNode (:,3),'o',...
      cubeShape(:,1),cubeShape(:,2),cubeShape(:,3),'-');
axis equal;

%% Set camera parameter
u_fov=1;
a_aspect=9/16;
FOV_H=2*pi*120/360;
v_fov=a_aspect*u_fov;
camF=u_fov/tan(FOV_H/2);
camFrame=[-u_fov,+u_fov,-v_fov,v_fov];

%% 
tend=700;
c1hist=zeros(tend,6);

%while(1)
    for t=1:tend
        
        xofs0=-5.0;
        xofs1=1.0;
        %xofs=t/tend*(xofs1-xofs0)+xofs0;
        %xofs=(-0.5*cos(t/tend*2*pi)+0.5)*(xofs1-xofs0)+xofs0;
        %yofs=-1.5; 
        xofs=1-3.0*cos(10*t/tend*2*pi);
        yofs=-3.0*sin(10*t/tend*2*pi);
        zofs=1.5+0.5*sin(35*t/tend*2*pi);
    
        %% Set camera position
        c0ofs=[   1,   0,  -1];
        
        c1ofs=[xofs,yofs,zofs];d1ofs=c0ofs-c1ofs;
        rxy1=norm(d1ofs(1,1:2));
        c1dir=[atan2(d1ofs(3),rxy1), atan2(d1ofs(2),d1ofs(1)), 0 ];
        c1=[c1ofs,c1dir];
        
        ep=0.1*[cos(c1dir(2)),-sin(c1dir(2)),0];
        c2ofs=[xofs+ep(1),yofs+ep(2),zofs];d2ofs=c0ofs-c2ofs;
        rxy2=norm(d2ofs(1,1:2));
        c2dir=[atan2(d2ofs(3),rxy2), atan2(d2ofs(2),d2ofs(1)), 0 ];
        c2=[c2ofs,c2dir];
        
        %% Rendaring 
        m1=world2cam(cubeNode,camF,c1);
        m1shape=m1(cubeShapeIdx,:);
        m2=world2cam(cubeNode,camF,c2);
        m2shape=m2(cubeShapeIdx,:);

        %% History
        c1hist(t,1:6)=c1;
        
        %% Plot
        subplot(2,2,1);
        plot( m1     (:,1), m1     (:,2), 'ob',...
              m1shape(:,1), m1shape(:,2), '-r');     
        axis equal;
        axis (camFrame);
        
        subplot(2,2,2);
        plot( m2     (:,1), m2     (:,2), 'ob',...
              m2shape(:,1), m2shape(:,2), '-r');              
        axis equal;
        axis (camFrame);

        subplot(2,1,2);
        plot3(cubeNode (:,1),cubeNode (:,2),cubeNode (:,3),'ob',...
              cubeShape(:,1),cubeShape(:,2),cubeShape(:,3),'-r',...
              c1hist   (1:t,1),c1hist(1:t,2),c1hist(1:t,3),'-g');             
        axis equal;

        drawnow ;
    end
%end