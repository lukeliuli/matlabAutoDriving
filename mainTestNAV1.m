function mainTestNAV1
%matlabµ¼º½Ä£¿éµÄgetting start
clear all;
close all;
addpath('D:\Documents\MATLAB\Examples\R2019b\shared_positioning\QuaternionExample\')
dr = HelperDrawRotation;
dr.drawTeapotRotations;

figure;
dr.draw3DOrientation(gca, [1/3 2/3 2/3], 30);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
q = quaternion(1,2,3,4);
p = quaternion(-5,6,-7,8);

p*q
q*p
p./q
p.\q

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ang = deg2rad(30);
q = quaternion(cos(ang/2), 0, 0, sin(ang/2));
pt = [0.7, 0.5, 0];  % Z-coordinate is 0 in the X-Y plane
ptrot = rotatepoint(q, pt)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ptframerot = rotateframe(q, pt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rotateframe(conj(q), ptframerot)
rotatepoint(q, ptframerot)
rad2deg(euler(qeul, 'ZYX', 'frame'))
eulerd(qeul, 'ZYX', 'frame')
rmat = rotmat(qeul, 'frame')
rotmatPoint = rotmat(q, 'point')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
euld = [30 20 -50];
dr.drawEulerRotation(gca, euld);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qeul = quaternion(deg2rad(euld), 'euler', 'ZYX', 'frame')
qeuld = quaternion(euld, 'eulerd', 'ZYX', 'frame')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rotmatPoint = rotmat(q, 'point')
rotmatPoint * (pt')

rotmatFrame = rotmat(q, 'frame')
rotmatFrame * (pt')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rv = rotvec(qeul)
quaternion(rv, 'rotvec')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


eul1 = [0, 10, 0];
eul2 = [0, 15, 0];
qdist1 = quaternion(deg2rad(eul1), 'euler', 'ZYX', 'frame');
qdist2 = quaternion(deg2rad(eul2), 'euler', 'ZYX', 'frame');

rad2deg(dist(qdist1, qdist2))

methods('quaternion')
end