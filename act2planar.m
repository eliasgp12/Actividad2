% Limpieza de pantalla
clear all
close all
clc

% Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) t l1 l2 l3

% Configuración del robot planar (3GDL) - Solo juntas rotacionales
RP=[0 0 0];

% Vector de coordenadas articulares
Q= [th1, th2, th3];
Qp= diff(Q, t); % Velocidades generalizadas

% Grado de libertad del robot
GDL= size(RP,2);

% Definición de matrices de transformación homogénea
P(:,:,1) = [l1*cos(th1); l1*sin(th1); 0];
R(:,:,1) = [cos(th1) -sin(th1) 0;
           sin(th1) cos(th1) 0;
           0       0       1];
P(:,:,2) = [l2*cos(th2); l2*sin(th2); 0];
R(:,:,2) = [cos(th2) -sin(th2) 0;
           sin(th2) cos(th2) 0;
           0       0       1];
P(:,:,3) = [l3*cos(th3); l3*sin(th3); 0];
R(:,:,3) = [cos(th3) -sin(th3) 0;
           sin(th3) cos(th3) 0;
           0       0       1];

Vector_Zeros = zeros(1,3);

% Inicialización de matrices homogéneas
for i = 1:GDL
    A(:,:,i) = simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
    if i == 1
        T(:,:,i) = A(:,:,i);
    else
        T(:,:,i) = simplify(T(:,:,i-1) * A(:,:,i));
    end
    RO(:,:,i) = T(1:3,1:3,i);
    PO(:,:,i) = T(1:3,4,i);
end

% Cálculo del Jacobiano
for k= 1:GDL
    if RP(k) == 0 % Articulación rotacional
        if k == 1
            Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));
            Jw_a(:,k) = [0;0;1];
        else
            Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k) = RO(:,3,k-1);
        end
    end
end

% Velocidades lineal y angular
V = simplify(Jv_a * Qp');
W = simplify(Jw_a * Qp');

disp('Velocidad lineal:');
pretty(V);
disp('Velocidad angular:');
pretty(W);
