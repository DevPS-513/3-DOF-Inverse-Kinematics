function [ px,py,pz] = project_along_vector( x1,y1,z1,x2,y2,z2,L)
% Solve for the point px,py,pz, that is
% a vector with magnitude L away in the direction between p2 and p1,
% starting at p1

% vector from point 1 to point 2
vx=x2-x1;
vy=y2-y1;
vz=z2-z1;
v=sqrt(vx^2+vy^2+vz^2);

ux=vx./v;
uy=vy./v;
uz=vz./v;

% Need to always project along radius

% Project backwards
px=x1+L.*ux;
py=y1+L.*uy;
pz=z1+L.*uz;

end

