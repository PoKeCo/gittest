shapeRaw2D=[120,220,205,230,205,220,210,195,215,200,215,240,235,260,275,260,...
         295,240,295,200,275,180,235,180,215,200,210,195,230,175,280,175,...
         300,195,305,220,305,240,560,240,560,220,570,195,575,200,575,240,...
         595,260,635,260,655,240,655,200,635,180,595,180,575,200,570,195,...
         590,175,640,175,660,195,670,220,670,235,720,235,740,215,735,175,...
         720,160,660,160,675,170,735,175,720,160,690,145,585,135,540,135,...
         555,155,550,225,400,225,405,170,395,135,540,135,395,135,385,85,...
         470,95,540,135,585,135,480,85,385,75,295,85,200,110,275,125,...
         265,140,335,225,400,225,405,170,395,135,275,125,395,135,385,85,...
         290,100,275,125,200,110,175,120,120,125,125,140,190,140,165,160,...
         135,155,125,140,115,180,120,220];
shapeSize2D=size(shapeRaw2D,2)/2;
shapeSideRev2D=reshape(shapeRaw2D,[2,shapeSize2D])';
shapeSide2D=[shapeSideRev2D(:,1),-shapeSideRev2D(:,2)];
shapeSideBtm=min(shapeSide2D(:,2));
shapeSide2D(:,2)=shapeSide2D(:,2)-shapeSideBtm;
shapeSideCenter=mean(shapeSide2D(:,1));
shapeSide2D(:,1)=shapeSide2D(:,1)-shapeSideCenter;
shapeOrgLength=max(shapeSide2D(:,1))-min(shapeSide2D(:,1));
shapeLength=4.675;
shapeWidth=1.820;
shapeWidthH=shapeWidth/2;
shapeSide2D=shapeSide2D*shapeLength/shapeOrgLength;

shapeW=shapeWidthH-max(0,shapeSide2D(:,2)-0.8)*0.3;

shapeSide3D=[shapeSide2D(:,1),+shapeW,shapeSide2D(:,2);
             shapeSide2D(:,1),-shapeW,shapeSide2D(:,2)];
%          shapeSide2D(end:-1:1,1),-shapeW(end:-1:1),shapeSide2D(end:-1:1,2)];

%shapeSkinIdx=[1:4,15:22,33:41,46:47,60:63,76:78,83,1];
shapeSkinIdx=[38:41,46:47,60:63,76:78,83,1];
shapeSkin2D=shapeSide2D(shapeSkinIdx,:);
shapeSkin2DSize=size(shapeSkin2D,1);

shapeSkin3D=zeros(shapeSkin2DSize*4,3);
for i=0:(shapeSkin2DSize-1)
    fl=i*2+1;
    fr=i*2+2;
    f =i+1;
    bl=shapeSkin2DSize*4+1-fl;
    br=shapeSkin2DSize*4+1-fr;
    b =f;%shapeSkin2DSize+1-f;

    Hf=shapeSkin2D(f,2);
    Wf=shapeWidthH-max(0,Hf-0.8)*0.3;
    Hb=shapeSkin2D(b,2);
    Wb=shapeWidthH-max(0,Hb-0.8)*0.3;
    if( mod(i,2)==0 )
        shapeSkin3D(fl,:)=[shapeSkin2D(f,1), +Wf, shapeSkin2D(f,2)];
        shapeSkin3D(fr,:)=[shapeSkin2D(f,1), -Wf, shapeSkin2D(f,2)];
        shapeSkin3D(bl,:)=[shapeSkin2D(b,1), -Wb, shapeSkin2D(b,2)];
        shapeSkin3D(br,:)=[shapeSkin2D(b,1), +Wb, shapeSkin2D(b,2)];
    else
        shapeSkin3D(fl,:)=[shapeSkin2D(f,1), -Wf, shapeSkin2D(f,2)];
        shapeSkin3D(fr,:)=[shapeSkin2D(f,1), +Wf, shapeSkin2D(f,2)];
        shapeSkin3D(bl,:)=[shapeSkin2D(b,1), +Wb, shapeSkin2D(b,2)];
        shapeSkin3D(br,:)=[shapeSkin2D(b,1), -Wb, shapeSkin2D(b,2)];
    end
end

%plot(shapeSide2D(:,1),shapeSide2D(:,2),'.-',...
%     shapeSkin2D(:,1),shapeSkin2D(:,2),'-');
%shapeSide3D=[shapeSide2D(:,1),+shapeWidthH*ones(shapeSize2D,1),shapeSide2D(:,2);
%             shapeSide2D(:,1),-shapeWidthH*ones(shapeSize2D,1),shapeSide2D(:,2)];
plot3(shapeSide3D(:,1),shapeSide3D(:,2),shapeSide3D(:,3),'-k',...
      shapeSkin3D(:,1),shapeSkin3D(:,2),shapeSkin3D(:,3),'-k');
%plot3(shapeSide3D(:,1),shapeSide3D(:,2),shapeSide3D(:,3),'-k');
%plot3(shapeSkin3D(:,1),shapeSkin3D(:,2),shapeSkin3D(:,3),'.-');
%shapeAll=[shapeSkin3D;shapeSide3D(end:-1:1,:)];
%plot3(shapeAll(:,1),shapeAll(:,2),shapeAll(:,3),'.-');
axis equal;
axis([-5,+5,-2,+2,-2,+2]);
view(30,30);
