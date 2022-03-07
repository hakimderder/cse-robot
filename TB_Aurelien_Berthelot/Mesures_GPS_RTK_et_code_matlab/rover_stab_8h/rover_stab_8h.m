%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% rover_stab_8h.m
% Aurélien Berthelot
% Code pour traiter la mesure de stabilité de 8h du rover
%
% ATTENTION, CE CODE UTILISE UNE FORMULE DE CALCUL DE DISTANCE ERRONEE
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;
load workspace;

% convertion degré minutes (dm) en degré decimale (dd)
data_8h_dd.Latitude = fix(data_8h_dm.Latitude/100) + (data_8h_dm.Latitude - fix(data_8h_dm.Latitude/100)*100)/60;
data_8h_dd.Longitude = fix(data_8h_dm.Longitude/100) + (data_8h_dm.Longitude - fix(data_8h_dm.Longitude/100)*100)/60;
data_8h_dd.Time = data_8h_dm.Time;

moy_latitude_dd = mean(data_8h_dd.Latitude);
moy_longitude_dd = mean(data_8h_dd.Longitude);

% definition de la grille d'affichage du graph
origine_x = 5.9642426;
step_x = 1e-7;
fin_x = 5.9642432;
origine_y = 46.1469236;
step_y = 1e-7;
fin_y = 46.1469244;

% calcul des distances de la grille
lat_1 = origine_y;
lat_2 = origine_y + step_y;

lon_1 = origine_x;
lon_2 = origine_x + step_x;

%distance d'un carré du plot en longitude
%depart  : lat_1     lon_1
%arrivee : lat_1     lon_2

distance_carre_horizontale = (60*acos(sin(lat_1)*sin(lat_1)+cos(lat_1)*cos(lat_1)*cos(lon_2-lon_1))) * 1851.85

%distance d'un carré du plot en latitude
%depart  : lat_1     lon_1
%arrivee : lat_2     lon_1

distance_carre_verticale = (60*acos(sin(lat_1)*sin(lat_2)+cos(lat_1)*cos(lat_2)*cos(lon_1-lon_1))) * 1851.85

markersize = 10;
linewidth = 3;

% affichage de la dispersion des points
figure();
hold on
p = plot(data_8h_dd.Longitude, data_8h_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p =plot(moy_longitude_dd,moy_latitude_dd,'xb');   
p.MarkerSize = markersize;
p.LineWidth=linewidth;

p =plot([origine_x origine_x+step_x],[origine_y origine_y],'r');  % longueur d'un carré
p.LineWidth=linewidth*2;
p =plot([origine_x origine_x],[origine_y origine_y+step_y],'b');  % largeur d'un carré
p.LineWidth=linewidth*2;
axis([origine_x fin_x origine_y fin_y])
legend('Mesures','Position moyenne',['Echelle : ~ ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ~',num2str(distance_carre_verticale*100),' cm'] )
xlabel('Longitude [°], format DD')
ylabel('Latitude [°], format DD')
title("Mesure de stabilité de la position du rover pendant 8h")
hold off
grid on;

%calcul de la distance entre chaque point et la moyenne des mesures
distance_point_moyenne = zeros(size(data_8h_dd.Latitude,1),1);
for j=1:size(data_8h_dd.Longitude)
    distance_point_moyenne(j) = (60*acos(sin(moy_latitude_dd)*sin(data_8h_dd.Latitude(j))+cos(moy_latitude_dd)*cos(data_8h_dd.Latitude(j))*cos(data_8h_dd.Longitude(j)-moy_longitude_dd))) *1851.85 ;
end

ecart_type = std(distance_point_moyenne) 

distance_max = max(distance_point_moyenne)

step_dist = 0.0005;
% histogamme des mesures
figure();
echelle_m = 0:step_dist:distance_max+0.002;
[ret_histc] = histc(distance_point_moyenne,echelle_m);
tot_points_on_graph = sum(ret_histc)
bar(echelle_m,ret_histc,'histc');
xlabel("Distance [m]")
ylabel("Nombre de mesures")
title("Dispersion des 28800 mesures par rapport à la position moyenne")
set(gca,'FontSize',15)
grid on;
grid minor

div_dist_pt_moy = reshape(distance_point_moyenne,3600,8);

%histogramme des mesures heures par heures
figure();
echelle_m = 0:0.0005:distance_max+0.002;
[ret_histc_2] = histc(div_dist_pt_moy,echelle_m);
b = bar(echelle_m,ret_histc_2,'stacked','FaceColor','flat');
b(8).CData = [0 0 0];
xlabel("Distance [m]")
ylabel("Nombre de mesures")
title("Dispersion des mesures par rapport à la position moyenne, divsée par heures")
set(gca,'FontSize',15)
legend("1re heure","2e heure","3e heure","4e heure","5e heure","6e heure","7e heure","8e heure")
grid on;
grid minor

dist_pre_neo_m8p = 0.025+step_dist;
dist_2_5_cm = dist_pre_neo_m8p/step_dist;
nb_pt_2_5_cm = (sum(ret_histc_2(1:dist_2_5_cm,:))')

dist_pre_neo_m8p = 0.004+step_dist;
dist_0_4_cm = dist_pre_neo_m8p/step_dist;
nb_pt_0_4_cm = (sum(ret_histc_2(1:dist_0_4_cm,:))')

