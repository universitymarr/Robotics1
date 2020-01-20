%% Find inverse kinematic (values of joints) of 3R robot in 3 dimension space (no planar)

syms q1 q2 q3

%% values of a2,a3,d1 
d1=0.5
a2=1.5
a3=1

%% define p
p=[cos(q1)*(a2*cos(q2)+a3*cos(q2+q3));
   sin(q1)*(a2*cos(q2)+a3*cos(q2+q3));
   d1+a2*sin(q2)+a3*sin(q2+q3)];

%% x is vector of joint that we use to compute jacobian matrix
x=[q1 q2 q3];

%% Jacobian Matrix
Jac=jacobian(p,x);

%% coordinates of point that we have
px=1.5309;
py=1.5309;
pz=0.25;

%% find value of sin e cos of q3 positive and q3 negative
c3=(px^2+py^2+(pz-d11)^2-a22^2-a33^2)/(2*a22*a33);
s3p=sqrt(1-c3^2);
s3n=-sqrt(1-c3^2);

%% compute q3 positive 
q03p=atan2(s3p,c3)
pause;

%% compute q3 positive 
q03n=atan2(s3n,c3)
pause;

%% compute q1 positive
q01p=atan2(py,px)
pause;

%% compute q1 negative
q01n=atan2(-py,-px)
pause;

%% det matrix
detM=px^2+py^2+(pz-d11)^2;

% q2 depends of sign of q1 and q3
%% sin and cos of q2 pos pos (q1 pos and q3 pos)
c2pp=((a22+a33*c3)*(px*cos(q01p)+py*sin(q01p))+(a33*s3p)*(pz-d11))/detM;
s2pp=((-a33*s3p)*(px*cos(q01p)+py*sin(q01p))+(a22+a33*c3)*(pz-d11))/detM;

%% sin and cos of q2 neg pos (q1 neg and q3 pos)
c2np=((a22+a33*c3)*(px*cos(q01n)+py*sin(q01n))+(a33*s3p)*(pz-d11))/detM;
s2np=((-a33*s3p)*(px*cos(q01n)+py*sin(q01n))+(a22+a33*c3)*(pz-d11))/detM;

%% sin and cos of q2 pos neg
c2pn=((a22+a33*c3)*(px*cos(q01p)+py*sin(q01p))+(a33*s3n)*(pz-d11))/detM;
s2pn=((-a33*s3n)*(px*cos(q01p)+py*sin(q01p))+(a22+a33*c3)*(pz-d11))/detM;

%% sin and cos of q2 neg neg
c2nn=((a22+a33*c3)*(px*cos(q01n)+py*sin(q01n))+(a33*s3n)*(pz-d11))/detM;
s2nn=((-a33*s3n)*(px*cos(q01n)+py*sin(q01n))+(a22+a33*c3)*(pz-d11))/detM;

%% q2 pos pos
q02pp=atan2(s2pp,c2pp);

%% q2 pos neg
q02pn=atan2(s2pn,c2pn);

%% q2 neg pos
q02np=atan2(s2np,c2np);

%% q2 neg neg
q02nn=atan2(s2nn,c2nn);

%% value of q = [q1 pos; q2 pos pos; q3 pos;]
q0pp=[q01p; q02pp; q03p;]
pause;

%% value of q = [q1 pos; q2 pos neg; q3 neg]
q0pn=[q01p; q02pn; q03n;]
pause;

%% value of q = [q1 neg; q2 neg pos; q3 pos;]
q0np=[q01n; q02np; q03p;]
pause;

%% value of q = [q1 neg; q2 neg neg; q3 neg;]
q0nn=[q01n; q02nn; q03n;]
pause;