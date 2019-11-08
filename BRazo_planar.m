%% Limpia la memoria de variables
clear all
close all
clc

%% Cierra y elimina cualquier objeto de tipo serial 
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Creación de un objeto tipo serial
arduino = serial('COM4','BaudRate',9600);
fopen(arduino);
if arduino.status == 'open'
    disp('Arduino conectado correctamente \n');
else
    disp('No se ha conectado el arduino \n');
    return
end
%% parámetros de medidas
tmax = 100;
rate = 33;

%Preguntar longitud del brazo
abc = 'introduzca el valor de l1:';
 L1= input(abc);
cba = 'introduzca el valor de l2:';
 l2= input(cba);
 
v1 = zeros(1,tmax*rate);
v2 = zeros(1,tmax*rate);
i = 1;
t = 0;

tic

 
  while 1
     clf
     printAxis();
     p1=[0;0;0];
       
 a= fscanf(arduino,'%d,%d');
 v1(i) = (a(1)-512)*130/512;
 v2(i) = (a(2)-512)*130/512;
 
 theta1_deg = v1(i);
 theta1_Rad = deg2rad(theta1_deg);
 theta2_deg = v2(i);
 theta2_Rad = deg2rad(theta2_deg);
  
TRz1 = [cos(theta1_Rad) -sin(theta1_Rad) 0 0;sin(theta1_Rad) cos(theta1_Rad) 0 0;0 0 1 0;0 0 0 1];
TRx1 = [1 0 0 L1;0 1 0 0; 0 0 1 0; 0 0 0 1];
T1=TRz1*TRx1;
p2=T1(1:3,4);
v1x=T1(1:3,1);
v1y=T1(1:3,2);
line([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'color',[0 0 0],'linewidth',5)%primer eslabon del brazo
line([p1(1) v1x(1)],[p1(2) v1x(2)],[p1(3) v1x(3)],'color',[1 0 0],'linewidth',5)%eje x1 pequeño
line([p1(1) v1y(1)],[p1(2) v1y(2)],[p1(3) v1y(3)],'color',[0 1 0],'linewidth',5)%eje y1 pequeño

TRz2 = [cos(theta2_Rad) -sin(theta2_Rad) 0 0;sin(theta2_Rad) cos(theta2_Rad) 0 0;0 0 1 0;0 0 0 1];
TRx2 = [1 0 0 l2;0 1 0 0; 0 0 1 0; 0 0 0 1];
T2=TRz2*TRx2;

Tf=T1*T2;
p3=Tf(1:3,4);
v3x=p3+Tf(1:3,1);
v3y=p3+Tf(1:3,2);
%v1x=TRz1(1:3,1);
%v1y=TRz1(1:3,2);

line([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'color',[0.6 0 0.8],'linewidth',5)%segundo eslabon del brazo
%line([p1(1) p2(1)],[p1(2) p2(2)],[p1(2) p2(2)],'color',[0 0 0],'linewidth',5)%primer eslabon del brazo
%line([p1(1) v1x(1)],[p1(2) v1x(2)],[p1(2) v1x(2)],'color',[1 0 0],'linewidth',5)%eje x1 pequeño
%line([p1(1) v1y(1)],[p1(2) v1y(2)],[p1(2) v1y(2)],'color',[0 1 0],'linewidth',5)%eje y1 pequeño

v2x=p2+T1(1:3,1);
v2y=p2+T1(1:3,2);
line([p2(1) v2x(1)],[p2(2) v2x(2)],[p2(3) v2x(3)],'color',[1 0 0],'linewidth',5)%eje x2 pequeño
line([p2(1) v2y(1)],[p2(2) v2y(2)],[p2(3) v2y(3)],'color',[0 1 0],'linewidth',5)%eje y2 pequeño
line([p3(1) v3x(1)],[p3(2) v3x(2)],[p3(3) v3x(3)],'color',[1 0 0],'linewidth',5)%eje x3 pequeño
line([p3(1) v3y(1)],[p3(2) v3y(2)],[p3(3) v3y(3)],'color',[0 1 0],'linewidth',5)%eje y3 pequeño

pause(0.01);
  end
 %% Limpiar '
fclose(arduino);
delete(arduino);
clear arduino;

 