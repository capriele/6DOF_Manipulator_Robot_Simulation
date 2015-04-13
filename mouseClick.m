%% Preparing MATLAB to start working

clc
clear all
close all
format compact
warning off

%% Carico i file di grafica (stl)

% aux = 100;   %variabile di scala
% base  = stlread('base.STL');   base.facecolor = [0 0 0];     base.edgecolor = 'none';  base.vertices = base.vertices / aux;
% link1 = stlread('link1.STL'); link1.facecolor = [.1 .1 .1]; link1.edgecolor = 'none'; link1.vertices = link1.vertices / aux;
% link2 = stlread('link2.STL'); link2.facecolor = [.2 .2 .2]; link2.edgecolor = 'none'; link2.vertices = link2.vertices / aux;
% link3 = stlread('link3.STL'); link3.facecolor = [.3 .3 .3]; link3.edgecolor = 'none'; link3.vertices = link3.vertices / aux;
% link4 = stlread('link4.STL'); link4.facecolor = [.4 .4 .4]; link4.edgecolor = 'none'; link4.vertices = link4.vertices / aux;
% link5 = stlread('link5.STL'); link5.facecolor = [.5 .5 .5]; link5.edgecolor = 'none'; link5.vertices = link5.vertices / aux;
% end_effector = stlread('end_effector.STL'); end_effector.facecolor = [.6 .6 .6]; end_effector.edgecolor = 'none'; end_effector.vertices = end_effector.vertices / aux;
aux = 100;   %variabile di scala
base  = stlread('base.STL');   base.facecolor = [ 1 0 0];     base.edgecolor = 'none';  base.vertices = base.vertices / aux;
link1 = stlread('link1.STL'); link1.facecolor = [.9 0 0]; link1.edgecolor = 'none'; link1.vertices = link1.vertices / aux;
link2 = stlread('link2.STL'); link2.facecolor = [.8 0 0]; link2.edgecolor = 'none'; link2.vertices = link2.vertices / aux;
link3 = stlread('link3.STL'); link3.facecolor = [.7 0 0]; link3.edgecolor = 'none'; link3.vertices = link3.vertices / aux;
link4 = stlread('link4.STL'); link4.facecolor = [.6 0 0]; link4.edgecolor = 'none'; link4.vertices = link4.vertices / aux;
link5 = stlread('link5.STL'); link5.facecolor = [.5 0 0]; link5.edgecolor = 'none'; link5.vertices = link5.vertices / aux;
end_effector = stlread('end_effector.STL'); end_effector.facecolor = [.6 .6 .6]; end_effector.edgecolor = 'none'; end_effector.vertices = end_effector.vertices / aux;

%% Lunghezze dei bracci (generati da SolidWorks design) e importati da https://github.com/adrianohrl/6DOF_Manipulator_Robot_Simulation

global l1 l2 d2 l3 d3 l4 l5 l6 robotCenter;
% 
% l1 = 1800 / aux;
% l2 = sqrt(500 ^ 2 + 3000 ^ 2) / aux;
% d2 = 125 / aux;
% l3 = 500 / aux;
% d3 = 125 / aux;
% l4 = 2500 / aux;
% l5 = 100 / aux;
% l6 = 300 / aux;

l1 = 1800 / aux;
l2 = sqrt(500 ^ 2 + 3000 ^ 2) / aux;
d2 = 125 / aux;
l3 = 500 / aux;
d3 = 125 / aux;
l4 = 2000 / aux;
l5 = 100 / aux;
l6 = 300 / aux;

%% Posiziono il robot nell'origine
% Le variabili *_new descrivono la posizione nuova assunta dai bracci,
% quelle *_cur la posizione corrente
% lastTheta: le variabili di giunto assunte nell'ultima posizione 
% workSpaceRadius: raggi dello spazio di lavoro
global base_new link1_cur link1_new link2_cur link2_new link3_cur link3_new link4_cur link4_new link5_cur link5_new end_effector_cur end_effector_new;
global lastTheta workSpaceRadius;
movePartsToOrigin();

theta = 0;
[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics(theta * ones(6, 1));
link1_cur = transformation(link1_new, T1_0);
link2_cur = transformation(link2_new, T2_0);
link3_cur = transformation(link3_new, T3_0);
link4_cur = transformation(link4_new, T4_0);
link5_cur = transformation(link5_new, T5_0);
end_effector_cur = transformation(end_effector_new, T6_0);

%% Utilizzo la cinematica inversa per rapresentare il robot
% *_priori: variabili di giunto iniziali (scelta arbitraria)
% startPoint: posizione inziale del polso 
% maxPoint: definisce le coordinate massime per un punto
global theta_2_priori theta_3_priori theta_4_priori theta_5_priori startPoint maxPoint;
theta_2_priori = theta * pi / 180;
theta_3_priori = theta * pi / 180;
theta_4_priori = theta * pi / 180;
theta_5_priori = theta * pi / 180;

figure(1);
cla, hold on %animo il grafico
cursor_handle = plot3(0,0,0,'r+');
axis equal;
max = l2 + l3 + l4 + l5 + l6;
min = -max;
maxPoint = 50;
workSpaceRadius = [max abs(l2 - l3)];
axis([min max min max 0 max]);%axis([x x y y z z]);
grid on;
view(30, 30), xlabel('x'), ylabel('y'), zlabel('z'); %ruoto il grafico di 30? lungo x e y
% aggiungo un callback al click del mouse
set(gca,'ButtonDownFcn', @mouseclick_callback)
set(get(gca,'Children'),'ButtonDownFcn', @mouseclick_callback)

startPoint = [10,10,40];
T = rettaPerDuePunti(startPoint, startPoint, 1); %risolvo cinematica diretta

J = [];

%Cinematica inversa
[theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T(:, :, 1));
J = cat(3, J, jacobianMatrix([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]));

%Cinematica diretta
lastTheta = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6];
[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]);
link1_cur = transformation(link1_new, T1_0);
link2_cur = transformation(link2_new, T2_0);
link3_cur = transformation(link3_new, T3_0);
link4_cur = transformation(link4_new, T4_0);
link5_cur = transformation(link5_new, T5_0);
end_effector_cur = transformation(end_effector_new, T6_0);

%Animazione
patch(base_new); 
patch(link1_cur); 
patch(link2_cur); 
patch(link3_cur); 
patch(link4_cur); 
patch(link5_cur); 
patch(end_effector_cur);

%Spazio di lavoro: ? formato da una corona circolare di raggi
%workSpaceRadius e viene rappresentato mediante due semisfere (verde e rossa)
[xs ys zs] = sphere; %inizializzo una sfera di raggio 1
xs = xs(11:end,:);       %taglio la sfera
ys = ys(11:end,:);       %taglio la sfera
zs = zs(11:end,:);       %taglio la sfera
workSpace = surf(xs*workSpaceRadius(1)+robotCenter(1),ys*workSpaceRadius(1)+robotCenter(2),zs*workSpaceRadius(1)+robotCenter(3),'EdgeColor','none','LineStyle','none','FaceLighting','phong'); 
set(workSpace,'FaceColor',[0 1 0],'FaceAlpha',0.2);%set(workSpace,'FaceColor',[red green blue],'FaceAlpha', trasparenza);
workSpace = surf(xs*workSpaceRadius(2)+robotCenter(1),ys*workSpaceRadius(2)+robotCenter(2),zs*workSpaceRadius(2)+robotCenter(3),'EdgeColor','none','LineStyle','none','FaceLighting','phong'); 
set(workSpace,'FaceColor',[1 1 0],'FaceAlpha',0.2);
