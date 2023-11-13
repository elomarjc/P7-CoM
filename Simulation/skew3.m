function U = skew3(uvec)
U = [0,-uvec(3),uvec(2);
     uvec(3),0,-uvec(1);
     -uvec(2),uvec(1),0];