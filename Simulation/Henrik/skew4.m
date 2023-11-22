function U = skew4(u_vec)
U = [        u_vec(4), u_vec(3),-u_vec(2), u_vec(1);
    -u_vec(3),        u_vec(4), u_vec(1), u_vec(2);
     u_vec(2),-u_vec(1),        u_vec(4), u_vec(3);
     -u_vec(1),-u_vec(2),-u_vec(3),        u_vec(4)];
end