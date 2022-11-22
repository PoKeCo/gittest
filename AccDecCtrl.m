G=9.8;      %[m/ss]

v0=60/3.6;  %[m/s]
v3=30/3.6;  %[m/s]
a_min=-0.2*G;%[m/ss]
J_min=-2.0; %[m/sss]
J_max=+2.0; %[m/sss]
DeltaT=0.1; %[s]

v=v0;
a=0;
J=0;

histSize=100;
figure(1);
hist=zeros(histSize,3);
trig_i=10;
base_i=0;
state=0;

for i=1:histSize
    if( i==trig_i )        
        state=state+1;
        switch(state)
            case 1
                k01=ceil(a_min/(J_min*DeltaT));
                T01=k01*DeltaT;
                J01=a_min/T01;

                J=J01;
                base_i=i;
                trig_i=i+k01;

            case 2
                a12=a;
                v1 =v;

                T23d=-a_min/J_max;
                v2d =v3-a12*T23d - 1/2 * J_min * T23d^2;
                k12 =floor((v2d-v1)/(a12*DeltaT));

                J=0;
                base_i=i;
                trig_i=i+k12;

            case 3
                v2=v;

                k23=ceil(-a12/(J_max*DeltaT));
                T23=k23*DeltaT;
                J23=-2/T23^2*(v3-v-a*T23);

                base_i=i;
                trig_i=i+k23;
                J=J23;

            case 4
                J=0;                
        end
    end
    if( state==3 )
        t=(i-base_i)*DeltaT;
        J23=-2/(T23-t)^2*(v3-v-a*(T23-t));
        %J=J23;
    end

    %Save Current State
    hist(i,:)=[v,a,J];

    %Update Plant State
    a=a+DeltaT*J;
    v=v+DeltaT*a;    

    % Plot 
    subplot(3,1,1);
    plot((1:i)',hist(1:i,1)*3.6,'.-');
    axis([1,histSize,0,60]);

    subplot(3,1,2);
    plot((1:i)',hist(1:i,2),'.-');
    axis([1,histSize,a_min,0.1]);

    subplot(3,1,3);
    plot((1:i)',hist(1:i,3),'.-');
    axis([1,histSize,J_min-0.1,J_max+0.1]);
    drawnow;
end



