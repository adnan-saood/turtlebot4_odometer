T = 40;
a = 0.75;
b = 1;

N = 400;
t = linspace(0,T,N)';
dt = T / N;

x = a + a * sin(2*pi* t / (T));
y = b * cos(2*pi*t / (T));

read_bag(1);

subplot(211);
plot(t,x)
hold on
plot(t,y)
title("World Frame");
subplot(212);
plot(x,y);

figure;

x_dot = diff(x)/dt;
y_dot = diff(y)/dt;
title("Vx, Vy")
plot(t(1:end-1), x_dot);
hold on;
plot(t(1:end-1), y_dot)

%%

V = sqrt(x_dot.^2 + y_dot.^2);

prev_angle = atan2(y_dot(1), x_dot(1));
Theta(1) = prev_angle;

for k = 2:length(x_dot)  % Assuming V_x and V_y are arrays
    current_angle = atan2(y_dot(k), x_dot(k));
    
    % Calculate the difference and adjust for wrapping
    angle_diff = current_angle - prev_angle;
    if angle_diff > pi
        angle_diff = angle_diff - 2*pi; 
    elseif angle_diff < -pi
        angle_diff = angle_diff + 2*pi;
    end
    
    % Update total angle
    Theta(k,1) = Theta(k-1) + angle_diff;
    
    % Update previous angle for next iteration
    prev_angle = current_angle;
end

figure
title("V,Omega")
subplot(211);
subtitle("V");
plot(t(1:end-1),V)
subplot(212);
title("Omega");

Omega = [0; diff(Theta)/dt];

plot(t(1:end-1), Omega);
Omegas = [t(1:end-1), Omega];
Vs = [t(1:end-1), V];


%% Compute Vl and Vr
l = 0.2;
Vlr = [1 -l/2; 1 l/2] * [V  Omega]';
Vlr = Vlr';
figure;

plot(t(1:end-1), Vlr);
Vl = Vlr(:,1);
Vr = Vlr(:,2);

Vls = [t(1:end-1), Vl];
Vrs = [t(1:end-1), Vr];
