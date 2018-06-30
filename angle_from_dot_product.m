function [ theta ] = angle_from_dot_product( a,b )


a_mag=sqrt(a(1)^2+a(2)^2+a(3)^2);
b_mag=sqrt(b(1)^2+b(2)^2+b(3)^2);

theta=acos((a_mag*b_mag)^-1*(a(1)*b(1)+a(2)*b(2)+a(3)*b(3)));

end

