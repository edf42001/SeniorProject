pkg load control

m = 0.1;  %mass of pendulum
l = .39;   %length of pendulum
g = 9.8;  %gravity
I = 1.0/3.0 * m * l^2;  %moment of inertia of pendulum
b = 0.1;  %damping factor

u = 0; %acceleration of cart

max_energy = l/2*m*g; %energy of pendulum when upright

phi_accel = 0;
phi_vel = 0.02;
phi = 3;
cart_accel = u;
cart_vel = 0;
cart = 0;

dt = 0.01;
P = [];
C = [];
t = 0:dt:10;
V = 0;
for i = t
  phi_accel = m*g*l/(2*I) * sin(phi) - b * phi_vel - m*l/(2*I) * cos(phi) * u; % equation of motion of pendulum
  phi_vel += phi_accel * dt;
  phi += phi_vel * dt;
  cart_accel = u;
  cart_vel += cart_accel * dt;
  cart += cart_vel * dt;
  
  P = [P, phi];  
  C = [C, cart];
  
  energy = (l/2 * m * g * (cos(phi)) + 0.5 * I * phi_vel^2); %0 is upright
  
%  r = 2.1; %cart position weighting factor
%  uA = 1.5; %max cart accel wanted
%  epsilon = 0.001; %numbers too small value
%  thingy = (sign(energy-max_energy) * phi_vel * cos(phi) - 2 * r * cart_vel);
%  numerator = uA * abs(thingy)  + 2 * r * cart * cart_vel;
%  
%  if(abs(thingy)>epsilon)
%    denominator = thingy;
%  else
%    denominator = epsilon*sign(thingy);
%  end
%  
%  u = numerator/denominator;
%  
%  if(abs(u)>100)
%    u
%    phi
%    thingy
%    numerator
%  end
%  
%  if(phi > 6.2 || phi < 0.08)
%    phi = 6.28;
%    cart = 0;
%    u = 0;
%  end
  u = 2.5*sign(energy-max_energy)*sign(cos(phi)) * sign(phi_vel) - 5*cart_vel-4*cart;
end

plotyy(t, P, t, C);