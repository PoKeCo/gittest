function [cam,rot,ofs]=world2cam(world,f,pos)

    x_start=  0;
    x_end  =500;   

    pos_size  =size(pos,2);
    world_size=size(world,1);
    %world_ofs=world-pos(1:3);
    ofs=pos(1:3);
    Pofs=repmat(ofs,[world_size,1]);
    if( pos_size>=4)
        cos_pitch=cos(-pos(4));
        sin_pitch=sin(-pos(4));
    else
        cos_pitch=1.0;
        sin_pitch=0.0;
    end
    Rpitch=[+cos_pitch,0,-sin_pitch;
                     0,1,         0;
            +sin_pitch,0,+cos_pitch]';

    if( pos_size>=5)
        cos_yaw  =cos(-pos(5));
        sin_yaw  =sin(-pos(5));
    else
        cos_yaw  =1.0;
        sin_yaw  =0.0;
    end
    Ryaw=[+cos_yaw,-sin_yaw,0;
          +sin_yaw,+cos_yaw,0;
                 0,       0,1]';          

    if( pos_size>=6)
        cos_roll  =cos(-pos(6));
        sin_roll  =sin(-pos(6));
    else
        cos_roll  =1.0;
        sin_roll  =0.0;
    end
    Rroll=[1,        0,        0;
           0,+cos_roll,-sin_roll;
           0,+sin_roll,+cos_roll]';
    
    rot=Ryaw*Rpitch*Rroll;    
    C=(world-Pofs)*rot;
    
    j=0;
    cam_size=0;
    cam_full=zeros(world_size,2);
    %idx_full=(1:world_size)';
    for i=1:world_size
        if( x_start<C(i,1) && C(i,1) <= x_end )
            j=j+1;     
            cam_full(j,1)=-f*C(i,2)/C(i,1);
            cam_full(j,2)=+f*C(i,3)/C(i,1);
            cam_size=j;
            %idx_full(j,1)=i;
        end
    end    
    if(cam_size~=0)
        cam=cam_full(1:cam_size,:);
        %idx=idx_full(1:cam_size,:);
    else
        cam=cam_full;
        %idx=idx_full;
    end
        
end