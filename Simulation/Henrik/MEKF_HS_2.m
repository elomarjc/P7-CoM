Td=0.1;
T=50;
N=ceil(T/Td);

q=zeros(4,N);
qhat=q;

%q(:,1) = [0,0,0.2,0.8]';   % Initial Rotation
q(:,1) = [0,0,0,1]'; 
q(:,1) = q(:,1)/norm(q(:,1));

%qhat(:,1) = [0,0,0.2,0.8]';   % Initial Rotation
qhat(:,1) = [0,0,0,1]';
qhat(:,1) = qhat(:,1)/norm(qhat(:,1));

Gyr = [0 0 1]'; %rotating around z-axis with angular velocity 1
GyrHat = Gyr + [0.1 0 0.6]'; %qyroscope measurement with error

for n=1:N-1
    q(:,n+1) =(eye(4,4)+Td/2*skew4([Gyr;0]))*q(:,n);
    q(:,n+1) = q(:,n+1)/norm(q(:,n+1)); % Normalized as modulus steadily grows.
    
    qhat(:,n+1) =(eye(4,4)+Td/2*skew4([GyrHat;0]))*qhat(:,n);
    qhat(:,n+1) = qhat(:,n+1)/norm(qhat(:,n+1)); % Normalized as modulus steadily grows.
    
    qmeas=q(:,n+1)+randn(4,1)*0.1;
    qmeas=qmeas/norm(qmeas);
    
    Ahat = (qhat(4,n+1)^2-norm(qhat(1:3,n+1))^2)*eye(3,3)+2*qhat(1:3,n+1)*qhat(1:3,n+1)'-2*qhat(4,n+1)*skew3(qhat(1:3,n+1)); % All ok
    A  = (q(4,n+1)^2-norm(q(1:3,n+1))^2)*eye(3,3)+2*q(1:3,n+1)*q(1:3,n+1)'-2*q(4,n+1)*skew3(q(1:3,n+1)); % All ok
    
    %measurements
    mg = A*[0  0 -1]'+randn(3,1)*0.01;
    mm = A*[1 0 0]'+randn(3,1)*0.01;
    
    %predicted measurements
    mghat = Ahat*[0  0 -1]';
    mmhat = Ahat*[1 0 0]';
    
    e1 = mg-mghat;
    e2 = mm - mmhat;
    x
    Ha = [skew3(mghat); skew3(mmhat)];
    
    %e=H*a, <e,e> = e'*e = a' * H' * H * a is to be mimimized wrt a =>
    %gradient = 2*H' * e = a

    
%     qd = skew4(qmeas)*[-qhat(1:3,n+1);qhat(4,n+1)]; %error quaternion
%     a = qd(1:3)/qd(4); %computing gibbs vector
    
     K=0.3;
     a=K*Ha' * [e1;e2]; %rudimentary Kalman gain

    %qd(1:3)=a*qd(4) and |qd(1:3) dq(4)|^2 = |qd(1:3)|^2 + qd(4)^2 =
    %|a*qd(4)|^2 + qd(4)^2 = |a|^2*qd(4)^2 + qd(4)^2 = (|a|^2 + 1)  qd(4)^2
    %= 1 => qd(4) = sqrt(1/(|a|^2 + 1))
    qd(4)=sqrt(1/(norm(a)^2+1)) %back to quaternion from scaled Gibbs vector
    qd(1:3)=a*qd(4);
    
    qhat(:,n+1) = skew4(qd)*qhat(:,n+1); %perception  update
    qhat(:,n+1) = qhat(:,n+1)/norm(qhat(:,n+1)); 

end
plot((1:N)*Td,q,(1:N)*Td,qhat,'*')


legend("q","qhat");
