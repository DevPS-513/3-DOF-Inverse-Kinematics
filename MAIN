% Initialize Matlab
clear all
close all
clc

% SI units
meters=1;
feet=0.3048;


ss=get(0,'ScreenSize');
sw=ss(3);
sh=ss(4);

% Initial angles
% all angles are taken within the plane that is parallel to all the links

    q1=45/360*2*pi;     % rotation of the base about the z axis
    q2=60/360*2*pi;     % rotation of first link relative to the base             
    q3=-30/360*2*pi;    % rotation of the second link relative to the first link     
    q4=-30/360*2*pi;    % rotation of

    l1=.02;             % z-axis distance from the global xyz frame to the first joint
                        % this code assumes the global frame of
                        % reference has its z-axis centered with the
                        % first joint
                            
    l2=3*feet;              % length of the first link
    l3=2*feet;              % length of the second link
    l4=1.5*feet;            % length of the third link

% Initial forward kinematics
% dh(theta,d,a,alpha)
% uses dh table convention which is a rotation about z by theta
% then a displacement along z by d
% then a displacement along x by a
% then a rotation about x by alpha
%
% T is a 4x4 transformation matrix
% T(1:3,1:3) is a rotation matrix
% T(1:3,4) is the x,y,z displacement of the frames origin
% T(4,:) is used for scaleing and remains at 0 0 0 1 for no scaleing

T10=dh(q1,l1,0,pi/2);   % Create transformation matrix from joint  1 to 0
T21=dh(q2,0,l2,0);      % Create transformation matrix from joint  2 to 1
T32=dh(q3,0,l3,0);      % Create transformation matrix from joint  3 to 2
T43=dh(q4,0,l4,0);      % Create transformation matrix from joint  4 to 3

T20=T10*T21;            % transformation matrix from frame 2 to global 0
T30=T20*T32;            % transformation matrix from frame 2 to global 0
T40=T30*T43;            % transformation matrix from frame 2 to global 0


% Initial position of each joint
p_joints=[ T10(1:3,4) T20(1:3,4) T30(1:3,4) T40(1:3,4)];

 x_joints_o= p_joints(1,:);
 y_joints_o= p_joints(2,:);
 z_joints_o= p_joints(3,:);
       
 x_joints= p_joints(1,:);
 y_joints= p_joints(2,:);
 z_joints= p_joints(3,:);
 
% Re-solve for initial points to check IK
% q1=q_IK,q2=q2_IK, ect...

% [q1_IK,q2_IK,q3_IK,q4_IK,p_joints_IK,iterations_IK] = inverseKinematics_fabrik(l1,l2,l3,l4,...
%                                                             x_joints_o,y_joints_o,z_joints_o,...
%                                                             x_joints_o(end),y_joints_o(end),z_joints_o(end),.001,100);
%  

% Time matrix for example helix path
N_cycles=3;
T=1;

dt=T/100;
Nt=round(N_cycles*T/dt);
t=[0:1:Nt-1]*dt;

x=1.7*cos(2*pi*t+pi/8);
y=1.7*sin(2*pi*t+pi/8);
z=-.05-.05*t;

        



% ROBOT FIGURE
f_2=figure(2);
    h.plot_joints=plot3(x_joints_o,y_joints_o,z_joints_o);
    hold on
    h.scatter_joints=scatter3(x_joints_o,y_joints_o,z_joints_o);
    hold on
    h.path=plot3(x,y,z);
    hold on
    h.task_point=scatter3(0,0,0);
    hold on
    h.q1=scatter3(0,0,0,'g','MarkerFaceColor','g');
    hold on
    h.q2=scatter3(0,0,0,'r','MarkerFaceColor','r');
    hold on
    h.q3=scatter3(0,0,0,'m','MarkerFaceColor','m');
    hold on
    h.q4=scatter3(0,0,0,'m','MarkerFaceColor','black');

     line([-.5 .5],[ 0 0],[0 0],'color','black')
         line([0 0],[ -.5 .5],[0 0],'color','black')

        xlabel('x')
        ylabel('y')
        zlabel('z')

    view([62 16])
    grid on

    h.task_point=scatter3(0,0,0,'black');
    legend([h.scatter_joints,h.q1,h.q2,h.q3,h.q4,h.path],'joints','joint q1','joint q2','joint q3','joint q4','path');
    
tol_limit=.005;


for n=1:Nt
   disp(['n is: ' num2str(n)]) 
% Call Inverse Kinematics function to solve for joint angles
[q1,q2,q3,q4,p_joints_IK,iterations] = inverseKinematics_fabrik(l1,l2,l3,l4,...
                                                            x_joints,y_joints,z_joints,...
                                                            x(n),y(n),z(n),tol_limit,100);

                        % Use forward kinmatics to move robot
                        % Initial forward kinematics
                        
T10=dh(q1,l1,0,pi/2);   % Create transformation matrix from 0 to 1
T21=dh(q2,0,l2,0);      % Create transformation matrix from 2 to 1
T32=dh(q3,0,l3,0);      % Create transformation matrix from 3 to 2
T43=dh(q4,0,l4,0);      % keep q4 constant

T20=T10*T21;            
T30=T20*T32;
T40=T30*T43;            % Transformation matrix from end effector to the global frame

p_joints=[ T10(1:3,4) T20(1:3,4) T30(1:3,4) T40(1:3,4)];

 x_joints= p_joints(1,:);
 y_joints= p_joints(2,:);
 z_joints= p_joints(3,:);
       

set(h.q1,'XDATA',p_joints(1,1),...
         'YDATA',p_joints(2,1),...
         'ZDATA',p_joints(3,1));
     
set(h.q2,'XDATA',p_joints(1,2),...
         'YDATA',p_joints(2,2),...
         'ZDATA',p_joints(3,2));
     
set(h.q3,'XDATA',p_joints(1,3),...
         'YDATA',p_joints(2,3),...
         'ZDATA',p_joints(3,3));  
     
     
set(h.q4,'XDATA',p_joints(1,4),...
         'YDATA',p_joints(2,4),...
         'ZDATA',p_joints(3,4));       
% joint_positions=[pos1 pos2 pos3];

    
set(h.plot_joints,'XDATA',p_joints(1,:),...
                   'YDATA',p_joints(2,:),...
                   'ZDATA',p_joints(3,:))
                   
set(h.scatter_joints,'XDATA',p_joints(1,:),...
               'YDATA',p_joints(2,:),...
               'ZDATA',p_joints(3,:))                
                   
set(h.task_point,'XDATA',x(n),...
                'YDATA',y(n),...
                'ZDATA',z(n))
            pause(.2)
end









