%% Compute pseudoinverse of Jacobian Matrix

function Jpse= pseudoinverse(J,m,n)
% J is Jacobian Matrix
% m is the dimension of space, for example if robot is planare we have m=2
% n is dimension of joint variables

rho= min(m,n)
if(rank(J) <= rho )
    Jt=J*transpose(J);
    Jpse = transpose(J)*inv(Jt);
else
    if(rank(J)>rho)
        Tj=transpose(J)*J;
        Jpse = inv(Tj)*transpose(J)
    end
end