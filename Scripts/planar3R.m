syms q1 q2 q3

%% Inverse kinematics of 3R Planar Robot

% length l
l1=1;
l2=1;
l3=1;

% x of p and y of p
px=1.646;
py=1.365;
  
%phi from rotation matrix
cosPhi = 2.25;
sinPhi = 0.7;

%computation of phi
fprintf("we compute phi from the rotation matrix\n");
%phi = atan2(sinPhi, cosPhi)+pi/2
phi = pi/4
fprintf("in deg: %.2f\n", rad2deg(phi))

% p in planar 3R
p = [l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3);
  l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3)]


fprintf("we first solve the joint variables q1, q2 in order to place\nthe position of the base of the third link in the necessary position\n");
p2 = [px; py] - [l3*cos(phi); l3*sin(phi)]
p2x = p2(1);
p2y = p2(2);

fprintf("thus a solution for q2 is given by:\n");
% second joint computations
c2=(p2x^2+p2y^2-l1^2-l2^2)/(2*l1*l2)
s2pos=sqrt(1-c2^2)
%other solution: -sqrt(1-c2^2)
s2neg=-sqrt(1-c2^2);

q02p=atan2(s2pos,c2)
q02n=atan2(s2neg,c2)

fprintf("and for q1:\n");
% first joint computations
detM=l1^2+l2^2+2*l1*l2*c2;

% positive solution of q1
s1pos=(p2y*(l1+l2*c2)-p2x*l2*s2pos)/detM
c1pos=(p2x*(l1+l2*c2)+p2y*l2*s2pos)/detM

% negative solution of q1
s1neg=(p2y*(l1+l2*c2)-p2x*l2*s2neg)/detM
c1neg=(p2x*(l1+l2*c2)+p2y*l2*s2neg)/detM

q01p=atan2(s1pos,c1pos)
q01n=atan2(s1neg,c1neg)

fprintf("and finally, q3 is recovered by phi-q1-q2\n");
q03p = phi-(q01p+q02p)
q03n = phi-(q01n+q02n)

fprintf("So, summing up, the two solutions are:\n")
% output positive
q0p=[q01p; q02p; q03p]

% output negative
q0n=[q01n; q02n; q03n]
