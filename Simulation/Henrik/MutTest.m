q = [0,0,0.2,0.8]';
qhat = [0,0,0.2,0.8]';
skew4(qhat)*q

qu=quaternion([q(4) q(1:3)']);
quhat=quaternion([qhat(4) qhat(1:3)']);
quhat*qu
