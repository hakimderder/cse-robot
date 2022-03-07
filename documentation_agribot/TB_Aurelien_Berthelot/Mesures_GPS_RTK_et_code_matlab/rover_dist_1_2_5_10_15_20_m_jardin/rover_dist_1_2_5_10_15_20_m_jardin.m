%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% rover_dist_1_2_5_10_15_20_m_jardin.m
% Aurélien Berthelot
% Code pour traiter les mesures de distances et de dispersion des points
% dans mon jardin (environemment avec des obstacles autour)
%
% ATTENTION, CE CODE UTILISE UNE FORMULE DE CALCUL DE DISTANCE ERRONEE
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear;
load workspace;

% convertion degré minutes (dm) en degré decimale (dd)
data_1m_dd.Latitude = fix(data_1m_dm.Latitude/100) + (data_1m_dm.Latitude - fix(data_1m_dm.Latitude/100)*100)/60;
data_1m_dd.Longitude = fix(data_1m_dm.Longitude/100) + (data_1m_dm.Longitude - fix(data_1m_dm.Longitude/100)*100)/60;
data_1m_dd.Time = data_1m_dm.Time;

data_2m_dd.Latitude = fix(data_2m_dm.Latitude/100) + (data_2m_dm.Latitude - fix(data_2m_dm.Latitude/100)*100)/60;
data_2m_dd.Longitude = fix(data_2m_dm.Longitude/100) + (data_2m_dm.Longitude - fix(data_2m_dm.Longitude/100)*100)/60;
data_2m_dd.Time = data_2m_dm.Time;

data_5m_dd.Latitude = fix(data_5m_dm.Latitude/100) + (data_5m_dm.Latitude - fix(data_5m_dm.Latitude/100)*100)/60;
data_5m_dd.Longitude = fix(data_5m_dm.Longitude/100) + (data_5m_dm.Longitude - fix(data_5m_dm.Longitude/100)*100)/60;
data_5m_dd.Time = data_5m_dm.Time;

data_10m_dd.Latitude = fix(data_10m_dm.Latitude/100) + (data_10m_dm.Latitude - fix(data_10m_dm.Latitude/100)*100)/60;
data_10m_dd.Longitude = fix(data_10m_dm.Longitude/100) + (data_10m_dm.Longitude - fix(data_10m_dm.Longitude/100)*100)/60;
data_10m_dd.Time = data_10m_dm.Time;

data_15m_dd.Latitude = fix(data_15m_dm.Latitude/100) + (data_15m_dm.Latitude - fix(data_15m_dm.Latitude/100)*100)/60;
data_15m_dd.Longitude = fix(data_15m_dm.Longitude/100) + (data_15m_dm.Longitude - fix(data_15m_dm.Longitude/100)*100)/60;
data_15m_dd.Time = data_15m_dm.Time;

data_20m_dd.Latitude = fix(data_20m_dm.Latitude/100) + (data_20m_dm.Latitude - fix(data_20m_dm.Latitude/100)*100)/60;
data_20m_dd.Longitude = fix(data_20m_dm.Longitude/100) + (data_20m_dm.Longitude - fix(data_20m_dm.Longitude/100)*100)/60;
data_20m_dd.Time = data_20m_dm.Time;

%calcul des moyennes
moy_1m_lat_dd = mean(data_1m_dd.Latitude);
moy_1m_lon_dd = mean(data_1m_dd.Longitude);

moy_2m_lat_dd = mean(data_2m_dd.Latitude);
moy_2m_lon_dd = mean(data_2m_dd.Longitude);

moy_5m_lat_dd = mean(data_5m_dd.Latitude);
moy_5m_lon_dd = mean(data_5m_dd.Longitude);

moy_10m_lat_dd = mean(data_10m_dd.Latitude);
moy_10m_lon_dd = mean(data_10m_dd.Longitude);

moy_15m_lat_dd = mean(data_15m_dd.Latitude);
moy_15m_lon_dd = mean(data_15m_dd.Longitude);

moy_20m_lat_dd = mean(data_20m_dd.Latitude);
moy_20m_lon_dd = mean(data_20m_dd.Longitude);

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

%affichage vue générale
figure(1);
hold on
p = plot(data_1m_dd.Longitude, data_1m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_1m_lon_dd, moy_1m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p = plot(data_2m_dd.Longitude, data_2m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_2m_lon_dd, moy_2m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p = plot(data_5m_dd.Longitude, data_5m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_5m_lon_dd, moy_5m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p = plot(data_10m_dd.Longitude, data_10m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_10m_lon_dd, moy_10m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p = plot(data_15m_dd.Longitude, data_15m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_15m_lon_dd, moy_15m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p = plot(data_20m_dd.Longitude, data_20m_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p = plot(moy_20m_lon_dd, moy_20m_lat_dd, 'xb');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

xlabel('Longitude [°], format DD')
ylabel('Latitude [°], format DD')
title("Mesure de points distant de 1 à 20 m de la base")
legend('Mesures ','Moyenne')
hold off
grid on;

% axes du graphique
step_x = 0.0000001;
step_y = 00.0000001;
nb_step_x = 10;
nb_step_y = 10;

%affichage dispersions de chaque mesure
figure(2);

subplot(6,1,1);
x_start = 5.96424085;
y_start = 46.1469125;

lat_1 = y_start ;
lat_2 = lat_1 + 5*step_y;

lon_1 = x_start +0.5*step_y;
lon_2 = lon_1 + step_x;

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
title("Dispersion des points à 1 m")
legend('Mesures ','Moyenne',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )

subplot(6,1,2);
p = plot(data_2m_dd.Longitude, data_2m_dd.Latitude, 'ro',moy_2m_lon_dd, moy_2m_lat_dd, 'xb');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
x_start = 5.96424715;
y_start = 46.146905;
axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("Dispersion des points à 2 m")

subplot(6,1,3);
p = plot(data_5m_dd.Longitude, data_5m_dd.Latitude, 'ro',moy_5m_lon_dd, moy_5m_lat_dd, 'xb');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
x_start = 5.96426665;
y_start = 46.1468812;
axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("Dispersion des points à 5 m")

subplot(6,1,4);
p = plot(data_10m_dd.Longitude, data_10m_dd.Latitude, 'ro',moy_10m_lon_dd, moy_10m_lat_dd, 'xb');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
x_start = 5.96430095;
y_start = 46.1468390;
axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
ylabel('Latitude [°], format DD')
title("Dispersion des points à 10 m")

subplot(6,1,5);
p = plot(data_15m_dd.Longitude, data_15m_dd.Latitude, 'ro',moy_15m_lon_dd, moy_15m_lat_dd, 'xb');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
x_start = 5.96433045;
y_start = 46.1468033;
axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("Dispersion des points à 15 m")

subplot(6,1,6);
p = plot(data_20m_dd.Longitude, data_20m_dd.Latitude, 'ro',moy_20m_lon_dd, moy_20m_lat_dd, 'xb');
p(1).MarkerSize = markersize;
p(2).MarkerSize = markersize;
p(1).LineWidth = linewidth;
p(2).LineWidth = linewidth;
x_start = 5.96436205;
y_start = 46.1467643;
axis([x_start x_start+nb_step_x*step_x y_start  y_start+nb_step_y*step_y])
grid on
set(gca,'FontSize',15)
title("Dispersion des points à 20 m")
xlabel('Longitude [°], format DD')

% distance entre la moyenne a 1m et a 2m
distance_moyenne_1_2 = (60*acos(sin(moy_1m_lat_dd)*sin(moy_2m_lat_dd)+cos(moy_1m_lat_dd)*cos(moy_2m_lat_dd)*cos(moy_2m_lon_dd-moy_1m_lon_dd))) * 1851.85;

% distance entre la moyenne a 1m et a 5m
distance_moyenne_1_5 = (60*acos(sin(moy_1m_lat_dd)*sin(moy_5m_lat_dd)+cos(moy_1m_lat_dd)*cos(moy_5m_lat_dd)*cos(moy_5m_lon_dd-moy_1m_lon_dd))) * 1851.85;

% distance entre la moyenne a 1m et a 10m
distance_moyenne_1_10 = (60*acos(sin(moy_1m_lat_dd)*sin(moy_10m_lat_dd)+cos(moy_1m_lat_dd)*cos(moy_10m_lat_dd)*cos(moy_10m_lon_dd-moy_1m_lon_dd))) * 1851.85;

% distance entre la moyenne a 1m et a 15m
distance_moyenne_1_15 = (60*acos(sin(moy_1m_lat_dd)*sin(moy_15m_lat_dd)+cos(moy_1m_lat_dd)*cos(moy_15m_lat_dd)*cos(moy_15m_lon_dd-moy_1m_lon_dd))) * 1851.85;

% distance entre la moyenne a 1m et a 20m
distance_moyenne_1_20 = (60*acos(sin(moy_1m_lat_dd)*sin(moy_20m_lat_dd)+cos(moy_1m_lat_dd)*cos(moy_20m_lat_dd)*cos(moy_20m_lon_dd-moy_1m_lon_dd))) * 1851.85;

distance_moyenne = [distance_moyenne_1_2 distance_moyenne_1_5 distance_moyenne_1_10 distance_moyenne_1_15 distance_moyenne_1_20]

%calcul de la distance entre chaque point et la moyenne des mesures a
%chaque distances
distance_pt_moy_1m = zeros(size(data_1m_dd.Latitude));
    for i=1:size(data_1m_dd.Latitude,1)
        distance_pt_moy_1m(i) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(moy_1m_lat_dd)+cos(data_1m_dd.Latitude(i))*cos(moy_1m_lat_dd)*cos(data_1m_dd.Longitude(i)-moy_1m_lon_dd))) *1851.85;
    end
dis_max_moy_1m = max(distance_pt_moy_1m);
ecart_type_moy_1m = std(distance_pt_moy_1m);

distance_pt_moy_2m = zeros(size(data_2m_dd.Latitude));
    for i=1:size(data_2m_dd.Latitude,1)
        distance_pt_moy_2m(i) = (60*acos(sin(data_2m_dd.Latitude(i))*sin(moy_2m_lat_dd)+cos(data_2m_dd.Latitude(i))*cos(moy_2m_lat_dd)*cos(data_2m_dd.Longitude(i)-moy_2m_lon_dd))) *1851.85;
    end
dis_max_moy_2m = max(distance_pt_moy_2m);
ecart_type_moy_2m = std(distance_pt_moy_2m);

distance_pt_moy_5m = zeros(size(data_5m_dd.Latitude));
    for i=1:size(data_5m_dd.Latitude,1)
        distance_pt_moy_5m(i) = (60*acos(sin(data_5m_dd.Latitude(i))*sin(moy_5m_lat_dd)+cos(data_5m_dd.Latitude(i))*cos(moy_5m_lat_dd)*cos(data_5m_dd.Longitude(i)-moy_5m_lon_dd))) *1851.85;
    end
dis_max_moy_5m = max(distance_pt_moy_5m);
ecart_type_moy_5m = std(distance_pt_moy_5m);

distance_pt_moy_10m = zeros(size(data_10m_dd.Latitude));
    for i=1:size(data_10m_dd.Latitude,1)
        distance_pt_moy_10m(i) = (60*acos(sin(data_10m_dd.Latitude(i))*sin(moy_10m_lat_dd)+cos(data_10m_dd.Latitude(i))*cos(moy_10m_lat_dd)*cos(data_10m_dd.Longitude(i)-moy_10m_lon_dd))) *1851.85;
    end
dis_max_moy_10m = max(distance_pt_moy_10m);
ecart_type_moy_10m = std(distance_pt_moy_10m);

distance_pt_moy_15m = zeros(size(data_15m_dd.Latitude));
    for i=1:size(data_15m_dd.Latitude,1)
        distance_pt_moy_15m(i) = (60*acos(sin(data_15m_dd.Latitude(i))*sin(moy_15m_lat_dd)+cos(data_15m_dd.Latitude(i))*cos(moy_15m_lat_dd)*cos(data_15m_dd.Longitude(i)-moy_15m_lon_dd))) *1851.85;
    end
dis_max_moy_15m = max(distance_pt_moy_15m);
ecart_type_moy_15m = std(distance_pt_moy_15m);

distance_pt_moy_20m = zeros(size(data_20m_dd.Latitude));
    for i=1:size(data_20m_dd.Latitude,1)
        distance_pt_moy_20m(i) = (60*acos(sin(data_20m_dd.Latitude(i))*sin(moy_20m_lat_dd)+cos(data_20m_dd.Latitude(i))*cos(moy_20m_lat_dd)*cos(data_20m_dd.Longitude(i)-moy_20m_lon_dd))) *1851.85;
    end
dis_max_moy_20m = max(distance_pt_moy_20m);
ecart_type_moy_20m = std(distance_pt_moy_20m);

dis_max_moy = [dis_max_moy_1m dis_max_moy_2m dis_max_moy_5m dis_max_moy_10m dis_max_moy_15m dis_max_moy_20m]
ecart_type_moy = [ecart_type_moy_1m ecart_type_moy_2m ecart_type_moy_5m ecart_type_moy_10m ecart_type_moy_15m ecart_type_moy_20m]

step_dist = 0.0005;
nb_step_0_4_cm = 9;
nb_step_2_5_cm = 51;

% histogramme de la dispersion des mesures
figure(3);
subplot(6,1,1);
echelle_m = 0:step_dist:0.06;
[ret_histc] = histc(distance_pt_moy_1m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 1 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_1m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_1m = sum(ret_histc(1:nb_step_2_5_cm));

subplot(6,1,2);
[ret_histc] = histc(distance_pt_moy_2m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 2 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_2m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_2m = sum(ret_histc(1:nb_step_2_5_cm));

subplot(6,1,3);
[ret_histc] = histc(distance_pt_moy_5m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 5 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_5m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_5m = sum(ret_histc(1:nb_step_2_5_cm));

subplot(6,1,4);
[ret_histc] = histc(distance_pt_moy_10m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
ylabel("Nombre de mesures")
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 10 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_10m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_10m = sum(ret_histc(1:nb_step_2_5_cm));

subplot(6,1,5);
[ret_histc] = histc(distance_pt_moy_15m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 15 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_15m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_15m = sum(ret_histc(1:nb_step_2_5_cm));

subplot(6,1,6);
[ret_histc] = histc(distance_pt_moy_20m,echelle_m);
tot_points_on_graph = sum(ret_histc);
bar(echelle_m,ret_histc,'histc');
xlabel("Distance [m]")
title("Dispersion des 1200 mesures par rapport à  moyenne, distance : 20 m")
set(gca,'FontSize',15)
grid on;
grid minor
cercle_0_4_cm_20m = sum(ret_histc(1:nb_step_0_4_cm));
cercle_2_5_cm_20m = sum(ret_histc(1:nb_step_2_5_cm));

pourcent_cercle_0_4_cm = [cercle_0_4_cm_1m cercle_0_4_cm_2m cercle_0_4_cm_5m cercle_0_4_cm_10m cercle_0_4_cm_15m cercle_0_4_cm_20m]/1200*100
pourcent_cercle_2_5_cm = [cercle_2_5_cm_1m cercle_2_5_cm_2m cercle_2_5_cm_5m cercle_2_5_cm_10m cercle_2_5_cm_15m cercle_2_5_cm_20m]/1200*100

distance_1m_2m = zeros(size(data_1m_dd.Latitude,1), size(data_2m_dd.Latitude,1));
distance_1m_5m = zeros(size(data_1m_dd.Latitude,1), size(data_5m_dd.Latitude,1));
distance_1m_10m = zeros(size(data_1m_dd.Latitude,1), size(data_10m_dd.Latitude,1));
distance_1m_15m = zeros(size(data_1m_dd.Latitude,1), size(data_15m_dd.Latitude,1));
distance_1m_20m = zeros(size(data_1m_dd.Latitude,1), size(data_20m_dd.Latitude,1));

% distance de chaque points de départ (1m) vers chaque point d'arrivé (2,5,10,15,20 m)
for i=1:size(data_1m_dd.Latitude,1)
    for j=1:size(data_2m_dd.Latitude,1)
        distance_1m_2m(i,j) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(data_2m_dd.Latitude(j))+cos(data_1m_dd.Latitude(i))*cos(data_2m_dd.Latitude(j))*cos(data_2m_dd.Longitude(j)-data_1m_dd.Longitude(i)))) *1851.85;
        distance_1m_5m(i,j) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(data_5m_dd.Latitude(j))+cos(data_1m_dd.Latitude(i))*cos(data_5m_dd.Latitude(j))*cos(data_5m_dd.Longitude(j)-data_1m_dd.Longitude(i)))) *1851.85;
        distance_1m_10m(i,j) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(data_10m_dd.Latitude(j))+cos(data_1m_dd.Latitude(i))*cos(data_10m_dd.Latitude(j))*cos(data_10m_dd.Longitude(j)-data_1m_dd.Longitude(i)))) *1851.85;
        distance_1m_15m(i,j) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(data_15m_dd.Latitude(j))+cos(data_1m_dd.Latitude(i))*cos(data_15m_dd.Latitude(j))*cos(data_15m_dd.Longitude(j)-data_1m_dd.Longitude(i)))) *1851.85;
        distance_1m_20m(i,j) = (60*acos(sin(data_1m_dd.Latitude(i))*sin(data_20m_dd.Latitude(j))+cos(data_1m_dd.Latitude(i))*cos(data_20m_dd.Latitude(j))*cos(data_20m_dd.Longitude(j)-data_1m_dd.Longitude(i)))) *1851.85;

    end
end

dist_1m_2m = reshape(distance_1m_2m,[],1);
dist_1m_5m = reshape(distance_1m_5m,[],1);
dist_1m_10m = reshape(distance_1m_10m,[],1);
dist_1m_15m = reshape(distance_1m_15m,[],1);
dist_1m_20m = reshape(distance_1m_20m,[],1);
dist_pts = [dist_1m_2m dist_1m_5m dist_1m_10m dist_1m_15m dist_1m_20m];

ecart_type_dist_1m_2m =  std(dist_1m_2m);
ecart_type_dist_1m_5m =  std(dist_1m_5m);
ecart_type_dist_1m_10m = std(dist_1m_10m);
ecart_type_dist_1m_15m = std(dist_1m_15m);
ecart_type_dist_1m_20m = std(dist_1m_20m);

distance_max_1m_2m  = max(dist_1m_2m);
distance_max_1m_5m  = max(dist_1m_5m);
distance_max_1m_10m = max(dist_1m_10m);
distance_max_1m_15m = max(dist_1m_15m);
distance_max_1m_20m = max(dist_1m_20m);
distance_max = [distance_max_1m_2m distance_max_1m_5m distance_max_1m_10m distance_max_1m_15m distance_max_1m_20m];

distance_min_1m_2m  = min(dist_1m_2m);
distance_min_1m_5m = min(dist_1m_5m);
distance_min_1m_10m = min(dist_1m_10m);
distance_min_1m_15m = min(dist_1m_15m);
distance_min_1m_20m = min(dist_1m_20m);
distance_min = [distance_min_1m_2m distance_min_1m_5m distance_min_1m_10m distance_min_1m_15m distance_min_1m_20m];

step_dist = 0.0005;
dist = 0.5;

% histogramme des distances entres chaque points
figure(4);
tab_dist = [ 2-1 5-1 10-1 15-1 20-1];

for i=1:5

    start_ech = distance_min(:,i) - step_dist;
    stop_ech = distance_max(:,i) + step_dist;
    echelle_m = start_ech-step_dist:step_dist:stop_ech+step_dist;
    [ret_histc] = histc(dist_pts(:,i),echelle_m);

    % verification que tous les points sont dans l'histogramme 3600*3600
    tot_points_on_graph = sum(ret_histc)  ;

    % affichage
    subplot(5,1,i);
    bar(echelle_m,ret_histc,'stacked');
    xlabel("Distance [m]")
    if i == 3
        ylabel("Nombre de mesures")
    end
    title( ['Distance obtenu à parti du point situé a 1 m, soit une distance de : ', num2str(tab_dist(i)), ' m'])
    set(gca,'FontSize',15)
    grid on;
    grid minor

end
