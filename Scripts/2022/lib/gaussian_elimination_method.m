% Code from "Gauss elimination and Gauss Jordan methods using MATLAB"
% https://www.youtube.com/watch?v=kMApKEKisKE
%https://gist.github.com/esromneb/1d57b1d16d54cde37332
function output=gaussian_elimination_method(a)
    [m,n]=size(a);
    output = a;
    for j=1:m-1
        for z=2:m
            if a(j,j)==0
                t=a(j,:);
                a(j,:)=a(z,:);
                output(z,:)=t;
            end
        end
        for i=j+1:m
            output(i,:)=simplify(a(i,:)-a(j,:)*simplify(a(i,j)/a(j,j)));
        end
    end
end
%{
% Code from "Gauss elimination and Gauss Jordan methods using MATLAB"
% https://www.youtube.com/watch?v=kMApKEKisKE

a = [3 4 -2 2 2
    4 9 -3 5 8
    -2 -3 7 6 10
    1 4 6 7 2];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Gauss elimination method [m,n)=size(a);
[m,n]=size(a);
for j=1:m-1
    for z=2:m
        if a(j,j)==0
            t=a(j,:);a(j,:)=a(z,:);
            a(z,:)=t;
        end
    end
    for i=j+1:m
        a(i,:)=a(i,:)-a(j,:)*(a(i,j)/a(j,j));
    end
end
x=zeros(1,m);
for s=m:-1:1
    c=0;
    for k=2:m
        c=c+a(s,k)*x(k);
    end
    x(s)=(a(s,n)-c)/a(s,s);
end
disp('Gauss elimination method:');
a
x'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauss-Jordan method
[m,n]=size(a);
for j=1:m-1
    for z=2:m
        if a(j,j)==0
            t=a(1,:);a(1,:)=a(z,:);
            a(z,:)=t;
        end
    end
    for i=j+1:m
        a(i,:)=a(i,:)-a(j,:)*(a(i,j)/a(j,j));
    end
end

for j=m:-1:2
    for i=j-1:-1:1
        a(i,:)=a(i,:)-a(j,:)*(a(i,j)/a(j,j));
    end
end

for s=1:m
    a(s,:)=a(s,:)/a(s,s);
    x(s)=a(s,n);
end
disp('Gauss-Jordan method:');
a
x'
%}