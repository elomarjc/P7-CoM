axis_rotation = [0;0;1];
angle = 90; 


q = [sind(angle/2)*axis_rotation;cosd(angle/2)]
A = (q(4)^2-norm(q(1:3))^2)*eye(3,3)+2*q(1:3)*q(1:3)'-2*q(4)*skew3(q(1:3))
%%
A
skew4(q)*skew4([1;0;0;0])*conj_quat(q)
%%
clc
q = [0,0,0,1]'
for i = 1:100
    q = (eye(4)+1/50*skew4([1;0;0;0]))*q;
    q = q/norm(q)
    A = (q(4)^2-norm(q(1:3))^2)*eye(3,3)+2*q(1:3)*q(1:3)'-2*q(4)*skew3(q(1:3));
    A*eye(3,3)
end

function q = conj_quat(q1)
    q = [-q1(1:3);q1(4)];
end