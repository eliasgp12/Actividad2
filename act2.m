% Limpieza de pantalla
clear all
close all
clc

% Declaración de variables simbólicas
syms th1(t) th2(t) th3(t) t l1 l2 l3

% Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP = [0 0 0];

% Creamos el vector de coordenadas articulares
Q = [th1, th2, th3];

% Creamos el vector de velocidades generalizadas
Qp = diff(Q, t);

% Número de grados de libertad del robot
GDL = length(RP);

% Articulación 1 a 2
P(:,:,1) = [0; 0; l1];
R(:,:,1) = [cos(th1)  0  sin(th1);
            sin(th1)  0 -cos(th1);
            0         1        0];

% Articulación 2 a 3
P(:,:,2) = [l2*cos(th2); l2*sin(th2); 0];
R(:,:,2) = [cos(th2) -sin(th2)  0;
            sin(th2)  cos(th2)  0;
            0         0         1];

% Articulación 3
P(:,:,3) = [l3*cos(th3); l3*sin(th3); 0];
R(:,:,3) = [cos(th3) -sin(th3)  0;
            sin(th3)  cos(th3)  0;
            0         0         1];

% Vector de ceros
Vector_Zeros = zeros(1, 3);

% Inicialización de matrices homogéneas locales y globales
A(:,:,GDL) = sym(zeros(4,4));  % Se inicializa simbólicamente
T(:,:,GDL) = sym(zeros(4,4));  
PO(:,:,GDL) = sym(zeros(3,1)); 
RO(:,:,GDL) = sym(zeros(3,3)); 

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

% Inicialización del Jacobiano en simbólico
Jv_a = sym(zeros(3, GDL));  % Matriz de 3 filas y GDL columnas
Jw_a = sym(zeros(3, GDL));

% Cálculo del Jacobiano Analítico
for k = 1:GDL
    if RP(k) == 0  % Articulación rotacional
        if k == 1
            Jv_a(:,k) = cross([0;0;1], PO(:,:,GDL));  
            Jw_a(:,k) = [0;0;1];
        else
            Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL) - PO(:,:,k-1));
            Jw_a(:,k) = RO(:,3,k-1);
        end
    else  % Articulación prismática
        if k == 1
            Jv_a(:,k) = [0;0;1];
        else
            Jv_a(:,k) = RO(:,3,k-1);
        end
        Jw_a(:,k) = [0;0;0];
    end
end

Jv_a = simplify(Jv_a);
Jw_a = simplify(Jw_a);

% Cálculo de velocidad lineal y angular
V = simplify(Jv_a * Qp.');
W = simplify(Jw_a * Qp.');

disp('Velocidad lineal obtenida mediante el Jacobiano:');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano:');
pretty(W);
