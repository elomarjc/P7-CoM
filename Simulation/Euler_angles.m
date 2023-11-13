close all
clear all
clc

% Rotation X-Y-Z

Az = @(phi)   [  cos(phi), sin(phi),          0;
                -sin(phi), cos(phi),          0;
                        0,        0,          1];

Ay = @(theta) [cos(theta),        0,-sin(theta);
                        0,        1,          0;
               sin(theta),        0, cos(theta)];

Ax = @(psi)   [         1,        0,          0;
                        0, cos(psi),   sin(psi);
                        0,-sin(psi),   cos(psi)];

A_XYZ = @(phi,theta,psi) Az(psi)*Ay(theta)*Ax(phi);


R = A_XYZ(0,89/180*pi,0)

hold on
grid on

quiver3(0,0,0,R(1,1),R(1,2),R(1,3),'off',"LineStyle","-","Color","r");
quiver3(0,0,0,R(2,1),R(2,2),R(2,3),'off',"LineStyle","-","Color","g");
quiver3(0,0,0,R(3,1),R(3,2),R(3,3),'off',"LineStyle","-","Color","b");
quiver3(0,0,0,1,0,0,'off',"LineStyle","--","Color","r");
quiver3(0,0,0,0,1,0,'off',"LineStyle","--","Color","g");
quiver3(0,0,0,0,0,1,'off',"LineStyle","--","Color","b");
legend("x","y","z","X","Y","Z");