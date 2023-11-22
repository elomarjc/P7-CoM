function q_est = MEKF(Acc,Mag,Gyr,Td)
% Td is the time step
% Acc is the normalised vector of gravity.
% Mag is normalised, points to x coordinate
% Gyr  is in degrees/s, in body coordinates.
persistent q % q1,q2,q3,q4 with q4 as double
persistent x
persistent P
% Initialization
if isempty(q)
    q = [0,0,0.2,0.8]';   % Initial Rotation
    q = q/norm(q);
end
if isempty(x)
    x = [0,0,0,0,0,0]'; % x = [a,b], with b as bias of gyro
end
if isempty(P)
    P = 10*eye(6,6);            % assume unkown initial state
end
% Correct units:
Gyr = Gyr/180*pi; % Nor rads/s
% Update state
%x = x % No changes in the update step as "a" is always 0 at begginig
F = [-skew3(Gyr),-eye(3,3);
    zeros(3,6)];

G = [-eye(3,3),zeros(3,3);
    zeros(3,3),eye(3,3)];

Q = [diag([0.0149    0.0162    0.0127]),zeros(3,3);
   zeros(3,3),0.01*eye(3,3)]; % This uses the noise of gyro from data, and the
% other 3 represent the noise of the bias, which is unkown.

P = P+Td*(F*P+P*F'+G*Q*G');
%Note that I have corercted skew4 to be the corerct function for quaternion
q =(eye(4,4)+Td/2*skew4([Gyr;0]))*q; 
q = q/norm(q); % Normalized as modulus steadily grows.
A = (q(4)^2-norm(q(1:3))^2)*eye(3,3)+2*q(1:3)*q(1:3)'-2*q(4)*skew3(q(1:3)); % All ok
% Measurement state
Mag_est = A*[1,0,0]' ;
Mag;
Acc_est = A*[0,0,-1]' ; % Acc points to ground
Acc;
Ha = [skew3(Acc_est); skew3(Mag_est)]; % h() function is the identity
H = [Ha,zeros(6,3)];

R = 0.01*[eye(3,3),zeros(3,3); % No acc, mag noise used here, if you want them,
zeros(3,3),eye(3,3)]; % They are (mag, accel): [0.2967,0.1789,0.2126]*1e-4, [0.0515,0.0423,0.1177]1e-4
K = P*H'/(H*P*H'+R) 
norm(K)
%K = 0.2*[eye(3,3), zeros(3,3);zeros(3,6)];
innovation = [Acc;Mag]-[Acc_est;Mag_est];
x = x + K*(innovation) ;
a = x(1:3)
P = P - K*H*P;

%Reset state

q_err = [x(1:3)/2; 1-(norm(x(1:3))^2)/8] ;% Using the quadratic approximation
q = skew4(q)*q_err; % Note that I have corrected skew4 to be the correct operation
x(1:3) = zeros(3,1);

q_est = [q(4);q(1:3)]; % I change quaternion from scalar last to scalar first,
% As that is matlab's interpretation of quaternions (useful this way in
% simulink)
