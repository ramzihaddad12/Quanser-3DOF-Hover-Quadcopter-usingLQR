%% parameters

Kt=0.0036;
Kf=0.1188;
L=0.197;
Jp=0.0552;
Jr=0.0552;
Jy=0.11;

PO = 10;    % max overshoot
Ts = 1;     % settling time

%% original system

A=[0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0];

B=[0,0,0,0;0,0,0,0;0,0,0,0;-(Kt/Jy),-(Kt/Jy),(Kt/Jy),(Kt/Jy);...
    (Kf*L/Jp),-(Kf*L/Jp),0,0;0,0,(Kf*L/Jr),-(Kf*L/Jr)];

C=[1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0];

D=[0,0,0,0;0,0,0,0;0,0,0,0];

% Q= 50*C'*C;
% R = 0.01*diag([1 1 1 1]);
% [K,S,P]= lqr( A, B, Q, R ); 
% K

%% A,B,C,D for each decoupled state (x3)

% pitch
Ap = [0,1;0,0];
Bp = [0,0,0,0;Kf*L/Jp,-Kf*L/Jp,0,0];
Cp = [1,0];
Dp = [0];

% roll
Ar = Ap;
Br = [0,0,0,0;0,0,Kf*L/Jr,-Kf*L/Jr];
Cr = Cp;
Dr = Dp;

% yaw
Ay = Ap;
By = [0,0,0,0;-Kt/Jy,-Kt/Jy,Kt/Jy,Kt/Jy];
Cy = Cp;
Dy = Dp;

%% desired poles

[zeta,wn] = SecondOrderResponse(PO,Ts);
SP1 = -zeta*wn+j*wn*sqrt(1-zeta^2);
SP2 = -zeta*wn-j*wn*sqrt(1-zeta^2);

%% gains for decoupled system

% Qp = 50*Cp'*Cp;
% Rp = 0.01*diag([1 1 1 1]);
% [Kp,S,P] = lqr( Ap, Bp, Qp, Rp );
% Kp
% 
% Qr = 50*Cr'*Cr;
% Rr = 0.01*diag([1 1 1 1]);
% [Kr,S,P] = lqr( Ar, Br, Qr, Rr );
% Kr
% 
% Qy = 50*Cy'*Cy;
% Ry = 0.01*diag([1 1 1 1]);
% [Ky,S,P] = lqr( Ay, By, Qy, Ry );
% Ky

Kp = place(Ap,Bp,[SP1 SP2]);
Kr = place(Ar,Br,[SP1 SP2]);
Ky = place(Ay,By,[SP1 SP2]);

%% concatinated gain for MIMO

K = [Ky(:,1),Kp(:,1),Kr(:,1),Ky(:,2),Kp(:,2),Kr(:,2)]

