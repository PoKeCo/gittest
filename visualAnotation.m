addpath('../../pllib/');

W=2.75;
Lf=10;
Ln=5;
H=1.2;
Pt=deg2rad(0);
forcus=1;

p = [  Ln, +W/2, 0;
       Ln, -W/2, 0;
       Lf, -W/2, 0;
       Lf, +W/2, 0;
       Ln, +W/2, 0];


%f = [ 2, -1, -9/16+H;
%      2, -1, +9/16+H; 
%      2, +1, +9/16+H; 
%      2, +1, -9/16+H; 
%      2, -1, -9/16+H]; 
deltaX=1;
fX2=H/(9/16)*forcus;
fZ2b=0;
fZ2t=2*(9/16)*fX2/forcus;
fY2 =1*fX2/forcus;
fX1=H/(9/16)*forcus-deltaX;

f  = [ fX2, -fY2, fZ2b;
       fX2, -fY2, fZ2t; 
       fX2, +fY2, fZ2t; 
       fX2, +fY2, fZ2b; 
       fX2, -fY2, fZ2b]; 

%figure(1);
%plot(p(:,1),p(:,2),'.-');
%axis equal;

rgn0=[-1,-9/16;+1,+9/16];


%for deltaX=0:0.1:2.5

    c1=transCameraOfs(p,forcus,[0,0,H,Pt,0]);
    %f1=transCameraOfs(f,forcus,[0,0,H,Pt,0]);
    f1=transCameraOfs(f,forcus,[-deltaX,0,H,Pt,0]);
    c2=transCameraOfs(p,forcus,[deltaX,0,H,Pt,0]);
    %f2=transCameraOfs(f,forcus,[deltaX,0,H,Pt,0]);
    f2=transCameraOfs(f,forcus,[0,0,H,Pt,0]);
    
    X1=[c2(:,1),ones(5,1)];
    Y1=[c1(:,1)];
    AB1=X1\Y1;
    X2=[c2(:,2),ones(5,2)];
    Y2=[c1(:,2)];
    AB2=X2\Y2;
    c1h=zeros(size(c1));
    c1h(:,1)=c2(:,1)*AB1(1)+AB1(2);
    c1h(:,2)=c2(:,2)*AB2(1)+AB2(2);
    rgn1h=rgn0;
    rgn1h(:,1)=rgn0(:,1)*AB1(1)+AB1(2);
    rgn1h(:,2)=rgn0(:,2)*AB2(1)+AB2(2);
    rect0 =getRgn2Rect(rgn0);
    rect1h=getRgn2Rect(rgn1h);

    cm=transCameraOfs(p,1,[deltaX,0,H,Pt,0]);    
    figure(2);
    subplot(2,1,2);
    plot(cm(:,1),cm(:,2),'.-y',...
         c1(:,1),c1(:,2),'.-r',...
         f1(:,1),f1(:,2),'x:r',...         
         c2(:,1),c2(:,2),'.-b',...
         f2(:,1),f2(:,2),'x:b',...
         c1h(:,1),c1h(:,2),'--m',...
         rect1h(:,1),rect1h(:,2),'--m' );
    axis equal;
    axis([rgn0(1,1),rgn0(2,1),rgn0(1,2),rgn0(2,2)]);
   
    subplot(2,2,1);
    plot(c2(:,1),c2(:,2),'.-b',...
         f2(:,1),f2(:,2),'x:b'     );
    axis equal;
    axis([rgn0(1,1),rgn0(2,1),rgn0(1,2),rgn0(2,2)]);

    subplot(2,2,2);
    plot(c1(:,1),c1(:,2),'.-r',...
         f1(:,1),f1(:,2),'x:r'     ); 
    axis equal;
    axis([rgn0(1,1),rgn0(2,1),rgn0(1,2),rgn0(2,2)]);
    
    drawnow;

%end

%%%%%%%%%%%%%%%%%%%%%%%%
%% Estimation part
idxr=1;
idxe=1;
Vr2=c1(idxr,2);
Vr1=c2(idxr,2);
Ve1=f2(idxe,2);
Vv1=0.0;
Vv2=0.0;
f=forcus;
Ve2ans=f1(idxe,2);
Xr1fph=(f*f-Vv1*Vr1)/(Vv1+Vr1);
Xe1fph=(f*f-Vv1*Ve1)/(Vv1+Ve1);
Xr2fph=(f*f-Vv2*Vr2)/(Vv2+Vr2);
Xe2fph=Xr2fph-Xr1fph+Xe1fph;
Ve2=(f*f-Vv2*Xe2fph)/(Vv2+Xe2fph);

[Ve2ans ,Ve2]

