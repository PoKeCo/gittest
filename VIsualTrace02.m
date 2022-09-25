addpath('../../pllib/');

video_enable=0;
if(video_enable)
    video_handler=VideoWriter('VisualRun004.avi','Motion JPEG AVI');
    %video_handler.FrameRate=15*4;
    video_handler.FrameRate=10;
    open(video_handler);
end

if(1)
    % Set Oval Course
    %g_roads{1}=getOvalCourse(100,200,0.5);
    %course=getOvalCourse(100,400,2);
    curve   =100;
    straight=200;
    oval_base=getOvalCourse(curve,straight,2);
    g_base = getChopped(oval_base,2);
    course_size=size(g_base,1);
    g_course = getOfsPath( g_base, 3*sin([0:course_size-1]/(course_size-1)*2*pi*10) );
    g_roads{1}=g_course;
    
    %g_roads{1}=getOfsPath(g_roads{1},3*sin([0:course_size-1]/(course_size-1)*2*pi*20));
    g_roads{2}=getOfsPath(g_roads{1},-3.25/2*ones(course_size,1));
    g_roads{3}=getOfsPath(g_roads{1},+3.25/2*ones(course_size,1));
    g_roads{4}=getOfsPath(g_roads{1},+3.25/2*randn(course_size,1));

    g_guardrail = getChopped(g_course,5);
    guardrail_size=size(g_guardrail,1);
    g_roads{5}  = getOfsPath(g_guardrail, (-3.25/2-0.5)*ones(guardrail_size,1) );
    g_roads{6}  = getOfsPath(g_guardrail, (+3.25/2+0.5)*ones(guardrail_size,1) );

    
else
    % Naruto Course
    rl=100;
    rs=30;
    th=0;
    r=rl;
    cnt=1000;
    g_roads{1}=zeros(cnt,2);
    for i=1:cnt
        g_roads{1}(i,1)= r * cos(th);
        g_roads{1}(i,2)= r * sin(th);
        r = rl + (rs-rl)*(i-1)/(cnt-1);
        th=th + 10/r;
    end
end

%Init car object parameters
x  =g_roads{1}(1,1);
y  =g_roads{1}(1,2);
yaw=atan2( g_roads{1}(2,2)-g_roads{1}(1,2), g_roads{1}(2,1)-g_roads{1}(1,1) );
x = x-straight;
y = y+0*3.5;
Speed=60;
prev_car=initCarAccord(x,y,yaw, pi*0.00, Speed/3.6, 0,0);
prev_car.dt = 0.1;
curr_car=runCar(prev_car);

% Set Car Shape
%car_shape=getBoxShape(curr_car);
car_shape=getAccordShape;

%Error estimator
y_err=0;
rc_err_amp=0;
fbg=0.5;
% Sim Step Loop

% Hist
hist_size=400;
hist=zeros(hist_size,6);
l_prev_path=[[0:1]'*200,zeros(2,1)];

% Disp
axis_max = 1;
for i=1:hist_size
    
    % Update prev_car    
    hist(i,1)=curr_car.x;
    hist(i,2)=curr_car.y;
    
    [g_nearest,g_crop_end,g_crop_path]=getAheadClosed(g_roads{1},[curr_car.x,curr_car.y],50 );

    [g_left_nearest ,g_left_end ,g_left_line ]=getAheadClosed(g_roads{3},[curr_car.x,curr_car.y],50 );
    [g_right_nearest,g_right_end,g_right_line]=getAheadClosed(g_roads{2},[curr_car.x,curr_car.y],50 );

    
    %System1
    if(1)
        % Get Transform Matrix and Previous Palnned Path
        [ l_past_path, Rp, Rc, Pp, Pc ] = convPrev2Curr( l_prev_path, ...
                                                      [prev_car.x, prev_car.y, prev_car.yaw], ...
                                                      [curr_car.x, curr_car.y, curr_car.yaw]     );
  
        % Get Current Error
        err = (g_nearest(1,1:2)-[curr_car.x,curr_car.y])*Rc;        
        
        % Ideal Path
        [r,c]=size(g_crop_path);
        l_crop_path=(g_crop_path-repmat([curr_car.x,curr_car.y],r,1))*Rc;


        [lr,lc]=size(g_left_line);
        l_left_line =(g_left_line-repmat([curr_car.x,curr_car.y],lr,1))*Rc;
        [rr,rc]=size(g_right_line);
        l_right_line=(g_right_line-repmat([curr_car.x,curr_car.y],rr,1))*Rc;
        [pr,pc]=size(g_roads{4});
        l_pattern=(g_roads{4}-repmat([curr_car.x,curr_car.y],pr,1))*Rc;


        [gr,gc]=size(g_roads{5});
        l_guardrail_r=(g_roads{5}-repmat([curr_car.x,curr_car.y],gr,1))*Rc;
        l_guardrail_l=(g_roads{6}-repmat([curr_car.x,curr_car.y],gr,1))*Rc;

        
        % Add Recognition Error
        rc_err_size = size(l_crop_path,1);
        rc_err_ratio = 0.5 / 50 * l_crop_path(end,1) * 10;
        rc_err_freq  = 1.0;
        
        %rc_err_amp  = sin(i*rc_err_freq*2*pi*curr_car.dt) * rc_err_ratio;
        %rc_err_amp  = randn(1,1) * rc_err_ratio;
        rc_err_amp  = rc_err_amp + (  randn(1,1) * rc_err_ratio - rc_err_amp )*0.5;
        rc_err_ramp = [0:(rc_err_size-1)]'/(rc_err_size-1) ;
                
        rc_err_rand = randn(rc_err_size,1).* rc_err_ramp .* rc_err_ratio *0 ;
        
        rc_err_ofs =  0.0;
        %rc_err_ofs =  0.5 * sin(i/10*2*pi);
        %rc_err_ofs =  0.5 * randn(1,1);
        
        rc_err_vect = ones(rc_err_size,1)*rc_err_ofs + ( rc_err_ramp * rc_err_amp  ) +rc_err_rand;
        l_road_comp=getOfsPath( l_crop_path, rc_err_vect );
        
        %if( mod(i,10)~=1 )
        %    l_road_comp = l_past_path;
        %    rc_err_size = size(l_road_comp,1);
        %end
        
        % Mix with Previous Path
        %mix=1-[ 0:(rc_err_size-1) ]'/(rc_err_size-1);
        keep_range=1;
        trans_range=rc_err_size-keep_range;
        trans_t=[0:(trans_range-1)]'/(trans_range-1);
        trans_vect=0.5+0.5*cos(trans_t*pi);
        mix=[ones(keep_range,1); trans_vect];        
        l_mix_path = getPathZip( l_road_comp, l_past_path, mix );
        
        % Select destination path
        l_dst_path = l_crop_path;
        %l_dst_path = l_road_comp;
        %l_dst_path = l_mix_path;
        
        % Get Pure Pursit
        %ahead_time = 1.2;   
        ahead_time = 0.8;         
        ahead_dist = ahead_time * curr_car.v;
        [ahead_s,ahead_e,l_ahead_path]=getAhead(l_dst_path,[0,0], ahead_dist );
           
        % Ideal Path
        R  =[+cos(curr_car.yaw), +sin(curr_car.yaw);
             -sin(curr_car.yaw), +cos(curr_car.yaw)];        
        [r,c]=size(l_ahead_path);
        g_ahead_path=l_ahead_path*R+repmat([curr_car.x,curr_car.y],r,1);
        
        %Current Posision correct feedback
        %fb_fact = 1.0;
        %if( i==0 )
        y_err_lim  =3.5;
        y_err_gain =10.0*0;
        %elseif( mod(i, 150)==0 )
        %y_err_gain = y_err_gain * 2.0;
        %end
        
        y_err=max(-y_err_lim,min(y_err_lim,y_err+(ahead_s(2)-y_err)*1.0));
        dest_x = ahead_e(1);
        dest_y = ahead_e(2)  +y_err*y_err_gain +ahead_s(2)*0.0;        
        
        l_prev_path = l_dst_path;
        hist(i,3) = -err(2);
        hist(i,4) = rc_err_amp;
        hist(i,5) = curr_car.dlt/curr_car.dltLim * 720;
        
        
        % Speed Control for Curve 
        %dest_v = ( ( 1.0 - min( 0.5, abs( err(2) ) )/0.5 )*60 + 10 )/3.6;
        %curr_car.a = (dest_v - curr_car.v )*0.5;
        
        
    end
    %fbg=fbg+err(2)*0.1;
    % Pure Pusit
    %curr_car.dlt = curr_car.WB * 2 * dest_y / (dest_x^2+dest_y^2 ) ;
    %if( i < 2 )
    curr_car.dlt = atan2( curr_car.WB * 2 * (dest_y) , (dest_x^2+dest_y^2 ));
    %curr_car.dlt = atan2( curr_car.WB * 2 * dest_y , (dest_x^2+dest_y^2 ));
    %curr_car.dlt = atan2( curr_car.WB , 80 );
    %curr_car.dlt = asin( curr_car.WB / 100 );
    %end
    
    % Turning Curve
    [r,c]=size(l_ahead_path);
    l_turning_x  = l_ahead_path(:,1);
    if( abs(dest_y)<1e-20)
        e = 1e-20;
        turning_R = (dest_x^2 + e^2)/(2*e);
        l_turning_y  = zeros(r,1);
    else
        turning_R = (dest_x^2 + dest_y^2)/(2*dest_y);
        l_turning_R = abs(turning_R) * ones(r,1);
        l_turning_y  = sign(dest_y) * ( l_turning_R-sqrt(l_turning_R.^2 - l_turning_x.^2 ));
    end
    hist(i,6) = turning_R;
            
            
    %Draw Current
    show_legend = 0;
    if( 1 ) % Show Global View
        g_shape=getMoveTo( car_shape, [curr_car.x, curr_car.y, curr_car.yaw] );    
        figure(1);
        subplot(4,1,1);
        
        hold off;
        plot(g_roads{1}(:,1)  ,g_roads{1}(:,2)  ,'-.k',...
             g_roads{2}(:,1)  ,g_roads{2}(:,2)  ,'--k',...
             g_roads{3}(:,1)  ,g_roads{3}(:,2)  ,'--k',...
             g_ahead_path(:,1),g_ahead_path(:,2),'-g',...
             g_shape(:,1)     ,g_shape(:,2)     ,'-b' ,...
             hist(1:i,1),      hist(1:i,2), '.-r'                 );
        axis equal;        
        wx = 20*5;wy = 5*5;
        axis( [curr_car.x-wx/2, curr_car.x+wx/2, ...
               curr_car.y-wy/2, curr_car.y+wy/2      ]);
           
        %drawnow;
        if( show_legend )
            legend('Ideal Path',...
                   'Lane',...
                   'Lane',...
                   'Destination Path',...
                   'Ego car',...
                   'History');    
        end
    end
    
    if(1)
        if(1) % Show Local Path
            %figure(2);
            subplot(4,1,2);

            plot( l_past_path(:,1)   , l_past_path(:,2)   ,'.-g', ...
                  l_crop_path(:,1)   , l_crop_path(:,2)   ,'-.k', ...
                  l_left_line(:,1)   , l_left_line(:,2)   ,'--k', ...
                  l_right_line(:,1)  , l_right_line(:,2)  ,'--r', ...
                  l_road_comp(:,1)   , l_road_comp(:,2)   ,'o-r', ...
                  l_dst_path(:,1)    , l_dst_path(:,2)    ,'.-b', ...
                  l_ahead_path(end,1), l_ahead_path(end,2),'xb' , ...
                  l_turning_x        , l_turning_y        ,'--c', ...
                  dest_x             , dest_y             ,'xr'      );
            axis([-5,50,-10,+10]);        
            axis equal;
            %axis([-5,50,-30,+30]);        
            if( show_legend )
                legend('Prev. destination path',...
                       'Ideal path',...
                       'Road Complication path',...
                       'Curr. destination path',...
                       'Ahead');
            end
            %drawnow;   
        end

        if(0) % Show Error Graph
            %figure(3);
            subplot(4,2,5);
            time_span=500;
            vct=max(1,i-time_span):i;        
            plot(vct,hist(vct,4),'-m',...
                 vct,hist(vct,3),'-b'          );         
            if( show_legend )        
                legend('Road Complication Erro at 50m',...
                       'Car Position Error at 0m');
            end
            axis_max = 2;            
            axis([max(1,i-time_span),max(time_span,i),-axis_max,+axis_max]);
            %drawnow;
        end

        if(0) % Show Error Graph
            %figure(3);
            subplot(4,2,6);
            time_span=500;
            vct=max(1,i-time_span):i;        
            plot(vct,hist(vct,5),'-',...
                 vct,hist(vct,6),'-'     );
            if( show_legend )        
                legend('Ste. Angle ','Turning curveture');
            end
            axis([max(1,i-time_span),max(time_span,i),-720,+720]);
            drawnow;
        end

        if(1)
            subplot(4,1,3:4);
            %l_left_line
            cam_z = 0;%1.26*1;
            z_dst_path    = addFlatZ(l_dst_path , -cam_z);
            z_road_comp   = addFlatZ(l_road_comp ,-cam_z);
            z_left_line   = addFlatZ(l_left_line ,-cam_z-0.0);
            z_right_line  = addFlatZ(l_right_line,-cam_z+0.0);
            l_turning=[l_turning_x,l_turning_y];
            z_turning  = addFlatZ(l_turning,-cam_z);
            z_pattern     = addFlatZ(l_pattern,-cam_z);
            z_guardrail_r = addGuardRailZ(l_guardrail_r);
            z_guardrail_l = addGuardRailZ(l_guardrail_l);

%             z_pattern = [z_pattern+[0,0,0];
%                          z_pattern+[0,0,.5];
%                          z_pattern+[.5,0,.5];
%                          z_pattern+[.5,0,0];
%                          z_pattern+[0,.5,0];
%                          z_pattern+[0,.5,.5];
%                          z_pattern+[.5,.5,.5];
%                          z_pattern+[.5,.5,0] ];
            z_dest   =  [dest_x, dest_y, -cam_z];

            f=1;
            cam_ofs=[0,0,1.26,(-15+0*sin(i/10*pi*2))*pi/180 ,0*pi/180];
            cam_dst_path = transCameraOfs(z_dst_path ,f,cam_ofs);
            cam_road_comp = transCameraOfs(z_road_comp ,f,cam_ofs);
            cam_left  = transCameraOfs(z_left_line ,f,cam_ofs);
            cam_right = transCameraOfs(z_right_line,f,cam_ofs);
            cam_turning = transCameraOfs(z_turning,f,cam_ofs);
            cam_pattern = transCameraOfs(z_pattern,f,cam_ofs);
            cam_dest = transCameraOfs(z_dest,f,cam_ofs);
            cam_guardrail_r = transCameraOfsNL(z_guardrail_r,f,cam_ofs);
            cam_guardrail_l = transCameraOfsNL(z_guardrail_l,f,cam_ofs);

            %figure(3);
            plot( ...
                  cam_pattern(:,1), cam_pattern(:,2),'.c',...
                  cam_dst_path(:,1), cam_dst_path(:,2),'.m',...
                  cam_road_comp (:,1), cam_road_comp (:,2), '.g',...
                  cam_left (:,1), cam_left (:,2), '-k',...
                  cam_right(:,1), cam_right(:,2), '-r',...
                  cam_turning(:,1), cam_turning(:,2), '-b',...
                  cam_dest(:,1), cam_dest(:,2), 'xr', ...
                  cam_guardrail_l(:,1), cam_guardrail_l(:,2), '.-b', ...
                  cam_guardrail_r(:,1), cam_guardrail_r(:,2), '.-b' );        
            axis equal;
            cam_zoom=1;
            xlim([-sqrt(3)     *f/cam_zoom, +sqrt(3)     *f/cam_zoom]);
            ylim([-sqrt(3)*9/16*f/cam_zoom, +sqrt(3)*9/16*f/cam_zoom]);
        end


        %% Video Export
        if(video_enable )
           frame = getframe(gcf);
           writeVideo(video_handler,frame);
        end
    end
    
    %Update State   
    prev_car=curr_car;
    curr_car=runCar(prev_car);
    
    %pause(0.1);
    
end

%% Video Terminator
if(video_enable)
    close(video_handler);
end

