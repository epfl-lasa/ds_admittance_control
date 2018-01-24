clc
close all
clear


% parameters

dt = 0.005;
t = 0:dt:100;

Mf = 10;
Df = 2;
Kf = 0;

% variables

xf = 0;
dxf = 0;
dxxf = 0;

F_ext = 0;

E_t = 0;
P_h = 0;
P_ext = 0;

F_ext = 0;
F_h = 0;

P_drain = .05;

for k = 1:numel(t)-1
    
    if( k*dt < 50)
        F_ext(end+1) = 1+ .01*randn;
    else
        F_ext(end+1) = -1+ .01*randn;
    end
    
    
    dxxf(end+1) = ( -Df*dxf(end) - Kf*xf(end)   + F_ext(end) ) / Mf;
    dxf(end+1) = dxf(end) + dxxf(end) * dt;
    xf(end+1) = xf(end) + dxf(end) * dt;
    
    P_ext(end+1) = F_ext(end) * dxf(end);
    
    E_t(end+1) = E_t(end) + P_ext(end) - P_h(end) - P_drain;
    if(E_t(end) < 0), E_t(end) = 0; end
    
    
    if(E_t(end) > 50 )
        alpha = (E_t(end) -50)/2;
        alpha = min(alpha,1);
        P_h(end+1) =  alpha * P_ext(end);
    else
        P_h(end+1) = 0;
    end
    
    
    F_h(end+1) = P_h(end) / dxf(end);
    
    
    
    
    

end

figure;

subplot(3,2,1)
plot(t,F_ext)
xlabel('Time [s]')
ylabel('Force [Nm/s]')
grid on
title('External force')


subplot(3,2,2)
plot(t,dxf)
xlabel('Time [s]')
ylabel('Velocity [Nm/s]')
grid on
title('Simulated velocity')


subplot(3,2,3)
plot(t,P_ext)
xlabel('Time [s]')
ylabel('Power [Nm/s]')
grid on
title('Estimated external power')


subplot(3,2,4)
plot(t,E_t)
xlabel('Time [s]')
ylabel('Energy [Nm]')
grid on
title('Stored Energy in the tank')


subplot(3,2,5)
plot(t,P_h)
xlabel('Time [s]')
ylabel('Power [Nm/s]')
grid on
title('Estimated human power')


subplot(3,2,6)
plot(t,F_h)
xlabel('Time [s]')
ylabel('Force [Nm/s]')
grid on
title('Estimated human force')









