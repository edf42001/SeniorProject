pkg load control

m = 0.1;  %mass of pendulum
l = .39;   %length of pendulum
g = 9.8;  %gravity
I = 1.0/3.0 * m * l^2;  %moment of inertia of pendulum
b = 0.1;  %damping factor

u = 0; %acceleration of cart

max_energy = l/2*m*g; %energy of pendulum when upright

phi_accel = 0;
phi_vel = 0;
phi = 3;
cart_accel = u;
cart_vel = 0;
cart = 0;

dt = 0.01;
P = [];
C = [];
t = 0:dt:6.5;

for i = t
  phi_accel = m*g*l/(2*I) * sin(phi) - b * phi_vel - m*l/(2*I) * cos(phi) * u; % equation of motion of pendulum
  phi_vel += phi_accel * dt;
  phi += phi_vel * dt;
  cart_accel = u;
  cart_vel += cart_accel * dt;
  cart += cart_vel * dt;
  
  P = [P, phi];  
  C = [C, cart];
  
  energy = l/2 * m * g * cos(phi) + 1.0/2 * I * phi_vel^2;
  
  u = 2*sign(energy-max_energy)*sign(cos(phi)) * sign(phi_vel) - 4*cart_vel-4*cart;
  
end

plotyy(t, P, t, C);