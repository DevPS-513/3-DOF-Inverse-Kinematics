function [A]= dh(theta,d,a,alpha)


A=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta);
    sin(theta) cos(alpha)*cos(theta) -sin(alpha)*sin(theta) a*sin(theta); 
    0           sin(alpha)                     cos(alpha)              d; 
    0           0                               0                       1];


end

