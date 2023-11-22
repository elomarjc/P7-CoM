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
GyrHat = Gyr + [0 0 0.6]'; %qyroscope measurement with error

for n=1:N-1
    q(:,n+1) =(eye(4,4)+Td/2*skew4([Gyr;0]))*q(:,n);
    q(:,n+1) = q(:,n+1)/norm(q(:,n+1)); % Normalized as modulus steadily grows.
    
    qhat(:,n+1) =(eye(4,4)+Td/2*skew4([GyrHat;0]))*qhat(:,n);
    qhat(:,n+1) = qhat(:,n+1)/norm(qhat(:,n+1)); % Normalized as modulus steadily grows.
    
    qmeas=q(:,n+1)+randn(4,1)*0.1;
    qmeas=qmeas/norm(qmeas);
    
    qd = skew4(qmeas)*[-qhat(1:3,n+1);qhat(4,n+1)]; %error quaternion
    a = qd(1:3)/qd(4); %computing gibbs vector
    K=0.5;
    a = K*a; %rudimentary Kalman gain
    %qd(1:3)=a*qd(4) and |qd(1:3) dq(4)|^2 = |qd(1:3)|^2 + qd(4)^2 =
    %|a*qd(4)|^2 + qd(4)^2 = |a|^2*qd(4)^2 + qd(4)^2 = (|a|^2 + 1)  qd(4)^2
    %= 1 => qd(4) = sqrt(1/(|a|^2 + 1))
    qd(4)=sqrt(1/(norm(a)^2+1)) %back to quaternion from scaled Gibbs vector
    qd(1:3)=a*qd(4);
    
    qhat(:,n+1) = skew4(qd)*qhat(:,n+1); %perception  update
    qhat(:,n+1) = qhat(:,n+1)/norm(qhat(:,n+1)); 

end
plot((1:N)*Td,q,(1:N)*Td,qhat,'*')


