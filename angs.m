clear
close all
clc
warning off
%% Config
escala = 1;
rotacion = 0; % 3 gira 45deg
puntos = 10;
%% Constants
largo_eslabon_1 = 20;
largo_eslabon_2 = 30;
a_min = 1.332; % No cambiar 1.332
b_no_rot = 4.71; % No cambiar 4.71
centro_trebol = [25; 25];
%% Discretize the clover
syms theta_trebol;
a_trebol = a_min * escala;
b_trebol = b_no_rot + rotacion;
trebol_x(theta_trebol) = (a_trebol*(sin(4*theta_trebol+b_trebol)+6)).*cos(theta_trebol)+centro_trebol(1);
trebol_y(theta_trebol) = (a_trebol*(sin(4*theta_trebol+b_trebol)+6)).*sin(theta_trebol)+centro_trebol(2);
intervalo_theta_trebol = (2*pi)/(puntos);
theta_trebol_discreto = 0:intervalo_theta_trebol:2*pi;
trebol_discretizado_x = trebol_x(theta_trebol_discreto);
trebol_discretizado_y = trebol_y(theta_trebol_discreto);
%% Find the angs
[q1vec, q2vec] = get_angs_multi(largo_eslabon_1, largo_eslabon_2, trebol_discretizado_x, trebol_discretizado_y);
%% Convert to degrees with no decimals
degrees1 = zeros(size(q1vec));
degrees2 = zeros(size(q2vec));
for i = 1:numel(q1vec)
    degrees1(i) = fix(rad2deg(q1vec(i)));
    degrees2(i) = fix(rad2deg(q2vec(i)));
end
%% Format as an arduino array and show in cmd
% Assuming you have the vectors degrees1 and degrees2
configurations = [degrees1(:), degrees2(:)];  % Combine the vectors into a matrix

% Generate the formatted string
str = '// Array to store configurations\n';
str = [str 'int configurations[][2] = {\n'];

[numConfigurations, ~] = size(configurations);
for i = 1:numConfigurations
    str = [str '  {' num2str(configurations(i, 1)) ', ' num2str(configurations(i, 2)) '}, // ' num2str(i) '\n'];
end

str = [str '};'];

% Replace \n with line break sequence for MATLAB command window
str = strrep(str, '\n', newline);

% Print the string in the command window
disp(str);




% %% Format the angs to send over serial
% configString = sprintf('%d,%d;', [degrees1', degrees2'].')
% %% Serial comms
% COM='/dev/ttyACM0'; % Cambiar al com de su arduino
% delete(instrfind({'Port'},{COM}));
% arduinoSerial=serial(COM);
% fopen(arduinoSerial);
% pause(3);
% disp('Enviando Datos..');
% fprintf(arduinoSerial,configString)
% disp('Datos Enviados.')
% fclose(arduinoSerial); 
% delete(arduinoSerial);
%% Inverse kinematics functions
function [q1, q2] = get_angs(a1, a2, x, y)
    q2 = [acos((x^2+y^2-a1^2-a2^2)/(2*a1*a2)); -acos((x^2+y^2-a1^2-a2^2)/(2*a1*a2))];
    q1 = [atan2(y,x)-atan2((a2*sin(q2)),(a1+a2*cos(q2))); atan2(y,x)+atan2((a2*sin(q2)),(a1+a2*cos(q2)))];
    q1 = q1(1:2);
end

function [q1vec, q2vec] = get_angs_multi(a1, a2, x, y)
    q1vec = [];
    q2vec = [];
    for i = 1:length(x)
        [q1, q2] = get_angs(a1, a2, x(i), y(i));
        q1vec(end+1) = q1(2); % una de las respuestas se está ignorando
        q2vec(end+1) = q2(2);
    end
end