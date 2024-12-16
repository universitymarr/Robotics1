%conversion from gray code to binary
%from exam of 2021.01.12 question 5

xgray=[0 1 1 0 1 0 0 1] %from MSB to LSB
nBits = 8;

xbin(1)=xgray(1);
for i=1:nBits-1
xbin(i+1)=xor(xbin(i),xgray(i+1));
end

xbin

% binary to decimal
xdec=xbin(nBits);

for i=1:nBits-1
xdec=xdec+xbin(nBits-i)*2^i;
end

xdec
