pkg load control

m = 0.1;  %mass of pendulum
l = .39;   %length of pendulum
g = 9.8;  %gravity
I = 1.0/3.0 * m * l^2;  %moment of inertia of pendulum
b = 0.1;  %damping factor

%%state matrices
%state is {theta, theta dot, cart position, position dot}
A = [0, 1, 0, 0;        %switch the pendulum from up to down by changing the sign of term (2,1)
     m*g*l/(2*I), -b, 0, 0;
     0, 0, 0, 1;
     0, 0, 0, 0];  
     
B = [0;
     -m*l/(2*I);
     0;
     1]; 
     
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0;
     0];
     
%%pole placement
Q = C'*C;
Q(1,1) = 900; %allowed error in pendulum angle
Q(3,3) = 2000;   %allowed error in cart pos
R = 1;
K = lqr(A, B, Q, R)
     
sys_op = ss(A, B, C, D);  %the system open loop
sys_cl = ss(A-B*K, B, C, D);  %the system closed loop

t = [0:0.01:5];
u = zeros(size(t));
x0 = [.04,0,0,0];   %start with pendulum at .8 rad

[y, t, x] = lsim(sys_cl, u, t, x0);  % run simulation
plotyy(t,y(:,1), t, y(:,2))
   