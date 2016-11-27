I = imread('Img.jpg');
I = rgb2gray(I);
imshow(I);

C = corner(I, 'MinimumEigenvalue', 40, 'QualityLevel', 0.4);
imshow(I);
hold on
plot(fineC(:,1), fineC(:,2), 'r*');


%Selected image corner coordinates that are detected with minimum eigenvalue corner detector.
fineC = [1161	2857;
2370	2698;
1661	1611;
1417	2279;
2361	2882;
2058	2412;
1540	2853;
2218	2272;
2390	2317;
1904	3022;
2060	2232;
2617	1732;
2403	2121;
2597	1948;
2052	2586;
1791	1985;
2083	1666;
1544	1930;
1664	2123;
2535	2761;
1930	1833;
1278	1625;
2202	2638;
2067	2045;
2382	2510;
2196	2819;
2237	1886;
1910	2532;
1026	3082;
2075	1857;
2244	1686;
1654	2599];

% World Coordinates that are calculated from the calibration object. The
% measurements are in mms.
WorldCor = [0 84 56;
168 0 112;
28 0 252;
0 28 140;
168 0 84;
112 0 140;
0 0 28;
140 0 168;
168 0 168;
84 0 28;
112 0 168;
196 0 252;
168 0 196;
196 0 224;
112 0 112;
56 0 196;
112 0 252;
0 0 196;
28 0 168;
196 0 112;
84 0 224;
0 56 252;
140 0 112;
112 0 196;
168 0 140
140 0 84;
140 0 224; 
84 0 112;
0 112 28;
112 0 224;
140 0 252;
28 0 84];

%The homogeneous matrix
n=32; 
P(1:2*n,1:12) = 0;
j=1;
for i=1:2:64
P(i,1) = WorldCor(j,1); P(i+1,5) = WorldCor(j,1);
P(i,2) = WorldCor(j,2); P(i+1,6) = WorldCor(j,2);
P(i,3) = WorldCor(j,3); P(i+1,7) = WorldCor(j,3);
P(i,4) = 1; P(i+1,8) = 1;
P(i,9:12) = P(i,1:4)*-1*fineC(j,1);
P(i+1,9:12) = P(i,1:4)*-1*fineC(j,2);
j = j+1;
end

%singular value decomposition to find 'm' which is the singular column of
%'V' with minimum values.
U=transpose(P)*P;
[V,D] = eig(U);
M=zeros(3,4);
j=0;
for i=1:1:3
    M(i,1)=V(j+1,1);
    M(i,2)=V(j+2,1);
    M(i,3)=V(j+3,1);
    M(i,4)=V(j+4,1);
    j=j+4;
end


A1(1) = M(1,1);
A1(2) = M(1,2);
A1(3) = M(1,3);

A2(1) = M(2,1);
A2(2) = M(2,2);
A2(3) = M(2,3);

A3(1) = M(3,1);
A3(2) = M(3,2);
A3(3) = M(3,3);

B = M(:,4);


f = norm(A3);
R3 = (1/f)*A3; % rotation matrix1
e = dot(A2,transpose(R3));
temp = A2 - e * R3; 
d = norm(temp);
R2 = (1/d)*temp; % rotation matrix2
c = dot(A1, transpose(R3));
b = dot(A1, transpose(R2));
temp2 = A1 -(b * R2) - (c * R3);
a = norm(temp2);
R1 = (1/a) * temp2; % rotation matrix3

% Normalizing values with respect to f since f= 1 on matrix K.
% These are Intrinsic parameters.
alpha = a / f;
skew = b / f;
u_0 = c / f;
beta_sinTheta = d / f;
v_0 = e / f;
f = f / f;
K =[alpha skew u_0;
    0 beta_sinTheta v_0;
    0 0 f;];  %Intrinsic Parameter Matrix.

theta = acos (-1*cross(A1,A3)*transpose(cross(A2,A3))/(norm(cross(A1,A3)*norm(cross(A2,A3))))); %theta comes out as 1.5 radians which is app. 86 degrees.
sinTheta = sin(theta);
cosTheta = cos(theta);

%Extrinsic Parameters
R1;
R2;
R3;
t = inv(K) * B; %Translation matrix