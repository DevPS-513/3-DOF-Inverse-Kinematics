function [q1,q2,q3,q4,p_joints,iterations] = inverseKinematics_fabrik(l1,l2,l3,l4,x_joints,y_joints,z_joints,x,y,z,tol_limit,max_iterations)
    % q1o,q2o,q3o, and q4o are the initial positions of each joint.
    % x y z is the desired location
    % tol is the tolerance between the desired location and actual one after
    % iteration

    % Find base rotation    
    
    % returns -pi to pi, different from atan
    q1_o=atan2(y_joints(1,end),x_joints(1,end)); % Initial angle of where end effector is
    q1=atan2(y,x);                               % Desired angle 
    
    base_rotation=q1-q1_o;                      % Base rotation      

    
    
    
    
    R_z=[cos(base_rotation)     -sin(base_rotation) 0;
         sin(base_rotation)     cos(base_rotation) 0;
         0                      0                   1]; % Base rotation matrix about z
 
 % Rotate the location of each joint by the base rotation
 % This will force the FABRIK algorithim to only solve 
 % in two dimensions, else each joint will move as if it has
 % a 3 DOF range of motion
 
    p4=R_z*[x_joints(1,4); y_joints(1,4); z_joints(1,4);];
    p3=R_z*[x_joints(1,3); y_joints(1,3); z_joints(1,3);];
    p2=R_z*[x_joints(1,2); y_joints(1,2); z_joints(1,2);];
    p1=R_z*[x_joints(1,1); y_joints(1,1); z_joints(1,1);];
 
% Store the (x,y,z) position of each joint    
    p4x=p4(1);
    p4y=p4(2);
    p4z=p4(3);
        
    p3x=p3(1);
    p3y=p3(2);
    p3z=p3(3);
            
            
    p2x=p2(1);
    p2y=p2(2);
    p2z=p2(3);
        
        
    p1x=p1(1);
    p1y=p1(2);
    p1z=p1(3);
        
 % store starting point of the first joint
    p1x_o=p1x;
    p1y_o=p1y;
    p1z_o=p1z;
 
 iterations=0;
for q=1:max_iterations
    
    % Make sure the desired x,y,z point is reachable
    if (sqrt(x^2+y^2+z^2)>(l2+l3+l4))
        disp (' desired point is likely out of reach')        
    end
    
   
% Overall algorithim: solve for unit vectors going backwards from the
% desired point to the original point of the joints, then forwards from the
% origin, see youtube video https://www.youtube.com/watch?v=UNoX65PRehA&t=817s


% backwards 
    %project_along_vector(x1,y1,z1,x2,y2,z2,L);
    
    [p3x,p3y,p3z]=project_along_vector(x,y,z,p3x,p3y,p3z,l4);
    [p2x,p2y,p2z]=project_along_vector(p3x,p3y,p3z,p2x,p2y,p2z,l3);
    [p1x,p1y,p1z]=project_along_vector(p2x,p2y,p2z,p1x,p1y,p1z,l2); 
    
% forwards

    [p2x,p2y,p2z]=project_along_vector(p1x_o,p1y_o,p1z_o,p2x,p2y,p2z,l2);
    [p3x,p3y,p3z]=project_along_vector(p2x,p2y,p2z,p3x,p3y,p3z,l3);
    [p4x,p4y,p4z]=project_along_vector(p3x,p3y,p3z,x,y,z,l4);

% Solve for tolerance between iterated point and desired x,y,z,
    tolx=p4x-x;
    toly=p4y-y;
    tolz=p4z-z;
% Make tolerance relative to x,y,z
    tol=sqrt(tolx^2+toly^2+tolz^2);

    iterations=iterations+1;
% Check if tolerance is within the specefied limit

    if tol<tol_limit
        break;
    end


end

% Re-organize points into a big matrix for plotting elsewhere
p_joints=  [[p1x p2x p3x p4x]; ...
            [p1y p2y p3y p4y]; ...
            [p1z p2z p3z p4z]; ];

% Return the joint angles by finding the angles with the dot produvt

v21=[p2x-p1x p2y-p1y p2z-p1z];
v32=[p3x-p2x p3y-p2y p3z-p2z];
v43=[p4x-p3x p4y-p3y p4z-p3z];




% returns -pi to pi
q2=atan2((p2z-p1z),sqrt((p2x-p1x)^2+(p2y-p1y)^2));

% Negative sign because of dh notation, a rotation away from the previous link
% and towards the x-y plane is a negative moment about the relative z axis.
% the relative z axis of each link is out of the page if looking at the arm
% in 2D
% the x axis in dh convention is typically along the link direction.


q3=-1*angle_from_dot_product(v21,v32);
q4=-1*angle_from_dot_product(v32,v43);


end