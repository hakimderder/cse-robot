%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% comp_dist_min_avec_sans_erreurs.m
% Aurélien Berthelot
% Code pour montrer l'impact des arrondis dans matlab
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear;
load workspace

% convertion degré minutes (dm) en degré decimale (dd)
data_1m_dd.Latitude = fix(data_1m_dm.Latitude/100) + (data_1m_dm.Latitude - fix(data_1m_dm.Latitude/100)*100)/60;
data_1m_dd.Longitude = fix(data_1m_dm.Longitude/100) + (data_1m_dm.Longitude - fix(data_1m_dm.Longitude/100)*100)/60;
data_1m_dd.Time = data_1m_dm.Time;

%calcul des moyennes
moy_1m_lat_dd = mean(data_1m_dd.Latitude);
moy_1m_lon_dd = mean(data_1m_dd.Longitude);

% axes du gragphique
origine_x = 5.9642562;
step_x = 2e-7;
fin_x = 5.9642578;
origine_y = 46.146922;
step_y = 1e-6;
fin_y = 46.146928;

% calcul des distances de la grille
lat_1 = origine_y;
lat_2 = origine_y + step_y;

lon_1 = origine_x;
lon_2 = origine_x + step_x;

%distance d'un carré du plot en longitude
%depart  : lat_1     lon_1
%arrivee : lat_1     lon_2

distance_carre_horizontale = (60*acos(sin(lat_1)*sin(lat_1)+cos(lat_1)*cos(lat_1)*cos(lon_2-lon_1))) * 1851.85;

%distance d'un carré du plot en latitude
%depart  : lat_1     lon_1
%arrivee : lat_2     lon_1

distance_carre_verticale = (60*acos(sin(lat_1)*sin(lat_2)+cos(lat_1)*cos(lat_2)*cos(lon_1-lon_1))) * 1851.85;

markersize = 10;
linewidth = 3;

step_x = 0.00000001;
step_y = 00.0000001;
nb_step_x = 19;
nb_step_y = 4;

% affichage et changement d'echelle a chaque graph (formule eronée)
figure();
subplot(6,1,1);
x_start = 5.95927565;
y_start = 46.1911451;

lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 4*step_x;

distance_carre_horizontale = (60*acos(sin(lat_1)*sin(lat_1)+cos(lat_1)*cos(lat_1)*cos(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acos(sin(lat_1)*sin(lat_2)+cos(lat_1)*cos(lat_2)*cos(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("1. Echelle longitude avec formule erronée, echelle longitude = 4 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )

subplot(6,1,2);
lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 2*step_x;

distance_carre_horizontale = (60*acos(sin(lat_1)*sin(lat_1)+cos(lat_1)*cos(lat_1)*cos(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acos(sin(lat_1)*sin(lat_2)+cos(lat_1)*cos(lat_2)*cos(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("2. Echelle longitude avec formule erronée, echelle longitude = 2 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )

subplot(6,1,3);
lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 1*step_x;

distance_carre_horizontale = (60*acos(sin(lat_1)*sin(lat_1)+cos(lat_1)*cos(lat_1)*cos(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acos(sin(lat_1)*sin(lat_2)+cos(lat_1)*cos(lat_2)*cos(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("3. Echelle longitude avec formule erronée, echelle longitude = 1 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )

% changement de formule (formule corrgiée)
subplot(6,1,4);
lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 1*step_x;

distance_carre_horizontale = (60*acosd(sind(lat_1)*sind(lat_1)+cosd(lat_1)*cosd(lat_1)*cosd(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acosd(sind(lat_1)*sind(lat_2)+cosd(lat_1)*cosd(lat_2)*cosd(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("4. Echelle longitude avec formule corrigée, echelle longitude = 1 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )

subplot(6,1,5);
lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 14*step_x;

distance_carre_horizontale = (60*acosd(sind(lat_1)*sind(lat_1)+cosd(lat_1)*cosd(lat_1)*cosd(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acosd(sind(lat_1)*sind(lat_2)+cosd(lat_1)*cosd(lat_2)*cosd(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("5. Echelle longitude avec formule corrigée, echelle longitude = 14 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )



subplot(6,1,6);
lat_1 = y_start +1*step_y ;
lat_2 = lat_1 + 2*step_y;

lon_1 = x_start + step_x;
lon_2 = lon_1 + 61*step_x;

distance_carre_horizontale = (60*acosd(sind(lat_1)*sind(lat_1)+cosd(lat_1)*cosd(lat_1)*cosd(lon_2-lon_1))) * 1851.85;
distance_carre_verticale = (60*acosd(sind(lat_1)*sind(lat_2)+cosd(lat_1)*cosd(lat_2)*cosd(lon_1-lon_1))) * 1851.85;

p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro',moy_1m_lon_dd, moy_1m_lat_dd, 'xb',[lon_1 lon_2], [lat_1 lat_1],'r' ,[lon_1 lon_1], [lat_1 lat_2],'b');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
p(3).LineWidth=linewidth*2;
p(4).LineWidth=linewidth*2;

axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("6. Echelle longitude avec formule corrigée, echelle longitude = 61 * 1e-8 deg")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )






