format long
%-------------------------------------------------------------------------%
% Paramètres du système 
l = 14.94872e-3; %metre
m = 0.52; %kg
M = 2*0.011; %kg
g = 9.81; %m.s-2
Lxx = 222.69522e-6; %kg.m^2
b = 0.1;
r = 42e-3; %m

q = (M+m)*(Lxx+m*l^2)-(m*l)^2;

% Parametre du moteur1
L1 = 2.8e-3; %Henri
R1 = 2.8; %ohm
f1 = 0.001; %N.m.s/rad
ke1 = 0.35; %V.s/rad
kt1 = ke1; %N.m/A
Cr1 = 0.075; % intensité de 200mA avant de commencé a tourner
Cr1max = ke1*1.576; % intensité a rotor bloquer sous 6V: 1.576A

y1 = ke1*kt1 + f1*R1;

Jm1=0.0025;

%Parametre du moteur 2
L2 = 2.9e-3; %Henri
R2 = 3.8;
f2 = 0.0003;
ke2 = 0.36;
kt2 = ke2;
Cr2 = 0.069;% intensié de 190mA avant de commencé a tourner
Cr2max = ke2*1.350; % intensité a rotor bloquer sous 6V: 1.35A

y2 = ke2*kt2 + f2*R2;

Jm2 = 0.0027;

% Tension maximal delivrable
Vmax = 6.00;

%Tension minimale qui fait agir le moteur
Vmin = 0.5;
