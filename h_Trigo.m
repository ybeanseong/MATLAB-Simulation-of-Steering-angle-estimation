function z_ret = h_Trigo(x)

C1 = (2*pi/296);
C2 = (2*pi/40);
C3 = (2*pi/1480);

z_ret = zeros(6,1);
z_ret(1) = sin(C1*(1480-x(1)));
z_ret(2) = cos(C1*(1480-x(1)));
z_ret(3) = sin(C2*(1480-x(1)));
z_ret(4) = cos(C2*(1480-x(1)));
z_ret(5) = sin(C3*(x(1)-x(3)));
z_ret(6) = cos(C3*(x(1)-x(3)));

end
