function [coff, A, b] = getCoff(waypoints,pol)
poly=pol*2;
n = size(waypoints, 1)-1; % number of segments P1..n
A = zeros(poly*n, poly*n);

b = zeros(1, poly*n);

% Example for first 4 equations:
for i=1: n
 b(1, i) = waypoints(i);
end
a=1;

for i=n+1:n*2
   b(1,i)= waypoints(a+1); 
   a=a+1;
end
for i=(n*2)+1:poly*n
    b(1,i)=0;
end

row = 1;
s=1;
for i=1: n
 A(row, s: poly*i) = polyT(poly, 0, 0);
 row = row + 1;
 s=s+poly;
end
s=1;
for i=1:n
  A(row, s:poly*i)= polyT(poly,0,1);
  row=row+1;
  s=s+poly;
end 

for i=1:pol-1
    A(row,1:poly)=polyT(poly,i,0);
    row=row+1;
end
 for i=1:pol-1
     A(row,((n-1)*poly)+1:n*poly)=polyT(poly,i,1);
     row = row+1;
 end
 
if (row-1~=poly*n)
 for i= 1:poly-2
     s=1;
     for j=1:pol-1
     A(row,s:j*poly)=polyT(poly,i,1);
     s=s+poly;
     A(row,s:(j+1)*poly)=-1*polyT(poly,i,0);
     row =row +1;
     end
     
 end
end







coff = inv(A)*b' ;
end
