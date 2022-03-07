%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% rover_distance_50_cm.m
% Aurélien Berthelot
% Code pour evaluer deux points distant de 50 cm
%
% ATTENTION, CE CODE UTILISE UNE FORMULE DE CALCUL DE DISTANCE ERRONEE
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear;
load workspace;
% convertion degré minutes (dm) en degré decimale (dd)
d_50cm_pt1_dd.Latitude = fix(d_50cm_pt1_dm.Latitude/100) + (d_50cm_pt1_dm.Latitude - fix(d_50cm_pt1_dm.Latitude/100)*100)/60;
d_50cm_pt1_dd.Longitude = fix(d_50cm_pt1_dm.Longitude/100) + (d_50cm_pt1_dm.Longitude - fix(d_50cm_pt1_dm.Longitude/100)*100)/60;
d_50cm_pt1_dd.Time = d_50cm_pt1_dm.Time;

% convertion degré minutes (dm) en degré decimale (dd)
d_50cm_pt2_dd.Latitude = fix(d_50cm_pt2_dm.Latitude/100) + (d_50cm_pt2_dm.Latitude - fix(d_50cm_pt2_dm.Latitude/100)*100)/60;
d_50cm_pt2_dd.Longitude = fix(d_50cm_pt2_dm.Longitude/100) + (d_50cm_pt2_dm.Longitude - fix(d_50cm_pt2_dm.Longitude/100)*100)/60;
d_50cm_pt2_dd.Time = d_50cm_pt2_dm.Time;

%calcul des moyennes
moy_latitude_pt1_dd = mean(d_50cm_pt1_dd.Latitude);
moy_longitude_pt1_dd = mean(d_50cm_pt1_dd.Longitude);

moy_latitude_pt2_dd = mean(d_50cm_pt2_dd.Latitude);
moy_longitude_pt2_dd = mean(d_50cm_pt2_dd.Longitude);

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

%affichage dispersion des points
figure();
hold on
p = plot(d_50cm_pt1_dd.Longitude, d_50cm_pt1_dd.Latitude, 'ro');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p =plot(d_50cm_pt2_dd.Longitude, d_50cm_pt2_dd.Latitude, 'bo');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p =plot(moy_longitude_pt1_dd,moy_latitude_pt1_dd,'xm');   
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p =plot(moy_longitude_pt2_dd,moy_latitude_pt2_dd,'gx');
p.MarkerSize = markersize;
p.LineWidth=linewidth;
p =plot([origine_x origine_x+step_x],[origine_y origine_y],'r');  % longueur d'un carré
p.LineWidth=linewidth*2;
p =plot([origine_x origine_x],[origine_y origine_y+step_y],'b');  % largeur d'un carré
p.LineWidth=linewidth*2;
axis([origine_x fin_x origine_y fin_y])
legend('Mesures de position du point 1','Mesures de position du point 2','Position moyenne du point 1','Position moyenne du point 2',['Echelle : ',num2str(distance_carre_horizontale*100),' cm'], ['Echelle : ',num2str(distance_carre_verticale*100),' cm'] )
xlabel('Longitude [°], format DD')
ylabel('Latitude [°], format DD')
title("Mesure d'une distance de 50 cm")
hold off
grid on;

% distance entre la moyenne des points 1 et 2
distance_moyenne = (60*acos(sin(moy_latitude_pt1_dd)*sin(moy_latitude_pt2_dd)+cos(moy_latitude_pt1_dd)*cos(moy_latitude_pt2_dd)*cos(moy_longitude_pt2_dd-moy_longitude_pt1_dd))) * 1851.85

%calcul de la distance entre chaque point et la moyenne des mesures
distance_pt_moy_et_pt_1 = zeros(size(d_50cm_pt1_dd.Latitude));
    for i=1:size(d_50cm_pt1_dd.Latitude,1)
        distance_pt_moy_et_pt_1(i) = (60*acos(sin(d_50cm_pt1_dd.Latitude(i))*sin(moy_latitude_pt1_dd)+cos(d_50cm_pt1_dd.Latitude(i))*cos(moy_latitude_pt1_dd)*cos(d_50cm_pt1_dd.Longitude(i)-moy_longitude_pt1_dd))) *1851.85;
    end
dis_max_moy_point_1 = max(distance_pt_moy_et_pt_1)
ecart_type_moy_pt1 = std(distance_pt_moy_et_pt_1)

distance_pt_moy_et_pt_2 = zeros(size(d_50cm_pt2_dd.Latitude));
    for i=1:size(d_50cm_pt2_dd.Latitude,1)
        distance_pt_moy_et_pt_2(i) = (60*acos(sin(d_50cm_pt2_dd.Latitude(i))*sin(moy_latitude_pt2_dd)+cos(d_50cm_pt2_dd.Latitude(i))*cos(moy_latitude_pt2_dd)*cos(d_50cm_pt2_dd.Longitude(i)-moy_longitude_pt2_dd))) *1851.85;
    end
dis_max_moy_point_2 = max(distance_pt_moy_et_pt_2)
ecart_type_moy_pt2 = std(distance_pt_moy_et_pt_2)

% distance de chaque points de départ vers chaque point d'arrivé
distance_pt_1_et_pt_2 = zeros(size(d_50cm_pt2_dd.Latitude,1), size(d_50cm_pt2_dd.Latitude,1));
for i=1:size(d_50cm_pt1_dd.Latitude,1)
    for j=1:size(d_50cm_pt2_dd.Latitude,1)
        distance_pt_1_et_pt_2(i,j) = (60*acos(sin(d_50cm_pt1_dd.Latitude(i))*sin(d_50cm_pt2_dd.Latitude(j))+cos(d_50cm_pt1_dd.Latitude(i))*cos(d_50cm_pt2_dd.Latitude(j))*cos(d_50cm_pt2_dd.Longitude(j)-d_50cm_pt1_dd.Longitude(i)))) *1851.85;
    end
end

dist_pts = reshape(distance_pt_1_et_pt_2,[],1);

ecart_type_dist = std(dist_pts)
dist_moyenne = mean(dist_pts)
distance_max = max(dist_pts)
distance_min = min(dist_pts)

% echelle histogramme
step_dist = 0.0005;
dist = 0.5;
start_ech = distance_min - step_dist;
stop_ech = distance_max + step_dist;
echelle_m = start_ech-step_dist:step_dist:stop_ech+step_dist;

% histogramme
[ret_histc] = histc(dist_pts,echelle_m);

% verification que tous les points sont dans l'histogramme 3600*3600
tot_points_on_graph = sum(ret_histc)  

% affichage de l'hitogramme des distances
figure();
bar(echelle_m,ret_histc,'stacked');
xlabel("Distance [m]")
ylabel("Nombre de mesures")
title("Distance obtenu à parti de chaque point de départ vers chaque point d'arrivée")
set(gca,'FontSize',15)
grid on;
grid minor

% calcul du pourcentage de points dans des cercles de rayon de 0.4 et 2.5 cm
nb_step_tot = round(( stop_ech + step_dist - start_ech + step_dist)/step_dist)
nb_step_50_cm = round(nb_step_tot /(stop_ech - step_dist - start_ech - step_dist) * (dist - start_ech ))

% calcul de la précision en fonction d'un rayon
nb_step_r_3_mm = 0.003/step_dist;
precision_3_mm = sum(ret_histc(nb_step_50_cm-nb_step_r_3_mm : nb_step_50_cm+nb_step_r_3_mm ))/tot_points_on_graph *100

nb_step_r_4_mm = 0.004/step_dist;
precision_4_mm = sum(ret_histc(nb_step_50_cm-nb_step_r_4_mm : nb_step_50_cm+nb_step_r_4_mm ))/tot_points_on_graph *100

nb_step_r_1_cm = 0.01/step_dist;
precision_1_cm = sum(ret_histc(nb_step_50_cm-nb_step_r_1_cm : nb_step_50_cm+nb_step_r_1_cm ))/tot_points_on_graph *100

nb_step_r_2_5_cm = 0.025/step_dist;
precision_2_5cm = sum(ret_histc(nb_step_50_cm-nb_step_r_2_5_cm : nb_step_50_cm+nb_step_r_2_5_cm ))/tot_points_on_graph *100



