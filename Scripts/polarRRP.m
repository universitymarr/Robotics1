syms q1 q2 q3  dq1 dq2 dq3

%% Inverse kinematics of RRP (Polar) Robot
% Slide of inverse kinematics

% length l
d = 0.5;

% x of p and y of p
px=1
py=1
pz=1

% direct kinematics
% px = q3*cos(q2)*cos(q1)
% py = q3*cos(q2)*sin(q1)
% pz = d + q3*sin(q2)

%third joint computation 
tmp=(px^2+py^2)+(pz-d)^2;
q3=sqrt(tmp);

if (q3<10^(-10))
  fprintf("There is a singularity\n")
  fprintf("q3=0, q1 and q2 undefined\n")
  return
endif

fprintf("we choose to take only the positive value of q3\n")

%second joint computation
s2 = (pz-d)/q3;
tmp = (px^2+py^2);
c2pos = sqrt(tmp)/q3;
c2neg = -sqrt(tmp)/q3;

q2pos=atan2(s2,c2pos);
q2neg=atan2(s2,c2neg);

if (tmp<10^(-10))
  fprintf("There is a singularity\n")
  fprintf("The two solutions are: \n")
  [q2pos; q3]
  fprintf("in deg:\n")
  d0p = [rad2deg(q2pos) rad2deg(q3)]
  [q2neg; q3]
  fprintf("in deg:\n")
  d0p = [rad2deg(q2neg) rad2deg(q3)]
  fprintf("q1 undefined\n")
  return
endif

%first joint computation
s1pos = py/c2pos;
s1neg = py/c2neg;

c1pos = px/c2pos;
c1neg = px/c2neg;

q1pos=atan2(s1pos, c1pos);
q1neg=atan2(s1neg, c1neg);

fprintf("the two solutions are:\n")
[q1pos; q2pos; q3]
fprintf("in deg:\n")
d0p = [rad2deg(q1pos) rad2deg(q2pos) rad2deg(q3)]
[q1neg; q2neg; q3]
fprintf("in deg:\n")
d0p = [rad2deg(q1neg) rad2deg(q2neg) rad2deg(q3)]
