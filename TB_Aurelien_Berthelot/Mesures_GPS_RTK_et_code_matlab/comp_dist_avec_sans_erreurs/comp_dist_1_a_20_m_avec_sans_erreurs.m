%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% comp_dist_1_a_20_m_avec_sans_erreurs.m
% Aurélien Berthelot
% Code pour comparer les précision avec les formules de calcul de distances
% éronée et corrgiée
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear; close all;

load workspace_jardin.mat
load workspace_sans_obstacles.mat

% convertion degré minutes (dm) en degré decimale (dd)
data_1m_dd_j.Latitude = fix(data_1m_dm_j.Latitude/100) + (data_1m_dm_j.Latitude - fix(data_1m_dm_j.Latitude/100)*100)/60;
data_1m_dd_j.Longitude = fix(data_1m_dm_j.Longitude/100) + (data_1m_dm_j.Longitude - fix(data_1m_dm_j.Longitude/100)*100)/60;
data_1m_dd_j.Time = data_1m_dm_j.Time;

data_2m_dd_j.Latitude = fix(data_2m_dm_j.Latitude/100) + (data_2m_dm_j.Latitude - fix(data_2m_dm_j.Latitude/100)*100)/60;
data_2m_dd_j.Longitude = fix(data_2m_dm_j.Longitude/100) + (data_2m_dm_j.Longitude - fix(data_2m_dm_j.Longitude/100)*100)/60;
data_2m_dd_j.Time = data_2m_dm_j.Time;

data_5m_dd_j.Latitude = fix(data_5m_dm_j.Latitude/100) + (data_5m_dm_j.Latitude - fix(data_5m_dm_j.Latitude/100)*100)/60;
data_5m_dd_j.Longitude = fix(data_5m_dm_j.Longitude/100) + (data_5m_dm_j.Longitude - fix(data_5m_dm_j.Longitude/100)*100)/60;
data_5m_dd_j.Time = data_5m_dm_j.Time;

data_10m_dd_j.Latitude = fix(data_10m_dm_j.Latitude/100) + (data_10m_dm_j.Latitude - fix(data_10m_dm_j.Latitude/100)*100)/60;
data_10m_dd_j.Longitude = fix(data_10m_dm_j.Longitude/100) + (data_10m_dm_j.Longitude - fix(data_10m_dm_j.Longitude/100)*100)/60;
data_10m_dd_j.Time = data_10m_dm_j.Time;

data_15m_dd_j.Latitude = fix(data_15m_dm_j.Latitude/100) + (data_15m_dm_j.Latitude - fix(data_15m_dm_j.Latitude/100)*100)/60;
data_15m_dd_j.Longitude = fix(data_15m_dm_j.Longitude/100) + (data_15m_dm_j.Longitude - fix(data_15m_dm_j.Longitude/100)*100)/60;
data_15m_dd_j.Time = data_15m_dm_j.Time;

data_20m_dd_j.Latitude = fix(data_20m_dm_j.Latitude/100) + (data_20m_dm_j.Latitude - fix(data_20m_dm_j.Latitude/100)*100)/60;
data_20m_dd_j.Longitude = fix(data_20m_dm_j.Longitude/100) + (data_20m_dm_j.Longitude - fix(data_20m_dm_j.Longitude/100)*100)/60;
data_20m_dd_j.Time = data_20m_dm_j.Time;

%calcul des moyennes
moy_1m_lat_dd_j = mean(data_1m_dd_j.Latitude);
moy_1m_lon_dd_j = mean(data_1m_dd_j.Longitude);

moy_2m_lat_dd_j = mean(data_2m_dd_j.Latitude);
moy_2m_lon_dd_j = mean(data_2m_dd_j.Longitude);

moy_5m_lat_dd_j = mean(data_5m_dd_j.Latitude);
moy_5m_lon_dd_j = mean(data_5m_dd_j.Longitude);

moy_10m_lat_dd_j = mean(data_10m_dd_j.Latitude);
moy_10m_lon_dd_j = mean(data_10m_dd_j.Longitude);

moy_15m_lat_dd_j = mean(data_15m_dd_j.Latitude);
moy_15m_lon_dd_j = mean(data_15m_dd_j.Longitude);

moy_20m_lat_dd_j = mean(data_20m_dd_j.Latitude);
moy_20m_lon_dd_j = mean(data_20m_dd_j.Longitude);

% convertion degré minutes (dm) en degré decimale (dd)
data_1m_dd_s_o.Latitude = fix(data_1m_dm_s_o.Latitude/100) + (data_1m_dm_s_o.Latitude - fix(data_1m_dm_s_o.Latitude/100)*100)/60;
data_1m_dd_s_o.Longitude = fix(data_1m_dm_s_o.Longitude/100) + (data_1m_dm_s_o.Longitude - fix(data_1m_dm_s_o.Longitude/100)*100)/60;
data_1m_dd_s_o.Time = data_1m_dm_s_o.Time;

data_2m_dd_s_o.Latitude = fix(data_2m_dm_s_o.Latitude/100) + (data_2m_dm_s_o.Latitude - fix(data_2m_dm_s_o.Latitude/100)*100)/60;
data_2m_dd_s_o.Longitude = fix(data_2m_dm_s_o.Longitude/100) + (data_2m_dm_s_o.Longitude - fix(data_2m_dm_s_o.Longitude/100)*100)/60;
data_2m_dd_s_o.Time = data_2m_dm_s_o.Time;

data_5m_dd_s_o.Latitude = fix(data_5m_dm_s_o.Latitude/100) + (data_5m_dm_s_o.Latitude - fix(data_5m_dm_s_o.Latitude/100)*100)/60;
data_5m_dd_s_o.Longitude = fix(data_5m_dm_s_o.Longitude/100) + (data_5m_dm_s_o.Longitude - fix(data_5m_dm_s_o.Longitude/100)*100)/60;
data_5m_dd_s_o.Time = data_5m_dm_s_o.Time;

data_10m_dd_s_o.Latitude = fix(data_10m_dm_s_o.Latitude/100) + (data_10m_dm_s_o.Latitude - fix(data_10m_dm_s_o.Latitude/100)*100)/60;
data_10m_dd_s_o.Longitude = fix(data_10m_dm_s_o.Longitude/100) + (data_10m_dm_s_o.Longitude - fix(data_10m_dm_s_o.Longitude/100)*100)/60;
data_10m_dd_s_o.Time = data_10m_dm_s_o.Time;

data_15m_dd_s_o.Latitude = fix(data_15m_dm_s_o.Latitude/100) + (data_15m_dm_s_o.Latitude - fix(data_15m_dm_s_o.Latitude/100)*100)/60;
data_15m_dd_s_o.Longitude = fix(data_15m_dm_s_o.Longitude/100) + (data_15m_dm_s_o.Longitude - fix(data_15m_dm_s_o.Longitude/100)*100)/60;
data_15m_dd_s_o.Time = data_15m_dm_s_o.Time;

data_20m_dd_s_o.Latitude = fix(data_20m_dm_s_o.Latitude/100) + (data_20m_dm_s_o.Latitude - fix(data_20m_dm_s_o.Latitude/100)*100)/60;
data_20m_dd_s_o.Longitude = fix(data_20m_dm_s_o.Longitude/100) + (data_20m_dm_s_o.Longitude - fix(data_20m_dm_s_o.Longitude/100)*100)/60;
data_20m_dd_s_o.Time = data_20m_dm_s_o.Time;

%calcul des moyennes
moy_1m_lat_dd_s_o = mean(data_1m_dd_s_o.Latitude);
moy_1m_lon_dd_s_o = mean(data_1m_dd_s_o.Longitude);

moy_2m_lat_dd_s_o = mean(data_2m_dd_s_o.Latitude);
moy_2m_lon_dd_s_o = mean(data_2m_dd_s_o.Longitude);

moy_5m_lat_dd_s_o = mean(data_5m_dd_s_o.Latitude);
moy_5m_lon_dd_s_o = mean(data_5m_dd_s_o.Longitude);

moy_10m_lat_dd_s_o = mean(data_10m_dd_s_o.Latitude);
moy_10m_lon_dd_s_o = mean(data_10m_dd_s_o.Longitude);

moy_15m_lat_dd_s_o = mean(data_15m_dd_s_o.Latitude);
moy_15m_lon_dd_s_o = mean(data_15m_dd_s_o.Longitude);

moy_20m_lat_dd_s_o = mean(data_20m_dd_s_o.Latitude);
moy_20m_lon_dd_s_o = mean(data_20m_dd_s_o.Longitude);

%% calcul distance jardin formule erronée

distance_moyenne_1_2_j = (60*acos(sin(moy_1m_lat_dd_j)*sin(moy_2m_lat_dd_j)+cos(moy_1m_lat_dd_j)*cos(moy_2m_lat_dd_j)*cos(moy_2m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_5_j = (60*acos(sin(moy_1m_lat_dd_j)*sin(moy_5m_lat_dd_j)+cos(moy_1m_lat_dd_j)*cos(moy_5m_lat_dd_j)*cos(moy_5m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_10_j = (60*acos(sin(moy_1m_lat_dd_j)*sin(moy_10m_lat_dd_j)+cos(moy_1m_lat_dd_j)*cos(moy_10m_lat_dd_j)*cos(moy_10m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_15_j = (60*acos(sin(moy_1m_lat_dd_j)*sin(moy_15m_lat_dd_j)+cos(moy_1m_lat_dd_j)*cos(moy_15m_lat_dd_j)*cos(moy_15m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_20_j = (60*acos(sin(moy_1m_lat_dd_j)*sin(moy_20m_lat_dd_j)+cos(moy_1m_lat_dd_j)*cos(moy_20m_lat_dd_j)*cos(moy_20m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;

distance_moyenne_j = [distance_moyenne_1_2_j distance_moyenne_1_5_j distance_moyenne_1_10_j distance_moyenne_1_15_j distance_moyenne_1_20_j]'

%% calcul distance jardin formule corrigée

distance_moyenne_1_2_j_corr = (60*acosd(sind(moy_1m_lat_dd_j)*sind(moy_2m_lat_dd_j)+cosd(moy_1m_lat_dd_j)*cosd(moy_2m_lat_dd_j)*cosd(moy_2m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_5_j_corr = (60*acosd(sind(moy_1m_lat_dd_j)*sind(moy_5m_lat_dd_j)+cosd(moy_1m_lat_dd_j)*cosd(moy_5m_lat_dd_j)*cosd(moy_5m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_10_j_corr = (60*acosd(sind(moy_1m_lat_dd_j)*sind(moy_10m_lat_dd_j)+cosd(moy_1m_lat_dd_j)*cosd(moy_10m_lat_dd_j)*cosd(moy_10m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_15_j_corr = (60*acosd(sind(moy_1m_lat_dd_j)*sind(moy_15m_lat_dd_j)+cosd(moy_1m_lat_dd_j)*cosd(moy_15m_lat_dd_j)*cosd(moy_15m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;
distance_moyenne_1_20_j_corr = (60*acosd(sind(moy_1m_lat_dd_j)*sind(moy_20m_lat_dd_j)+cosd(moy_1m_lat_dd_j)*cosd(moy_20m_lat_dd_j)*cosd(moy_20m_lon_dd_j-moy_1m_lon_dd_j))) * 1851.85;

distance_moyenne_j_corr = [distance_moyenne_1_2_j_corr distance_moyenne_1_5_j_corr distance_moyenne_1_10_j_corr distance_moyenne_1_15_j_corr distance_moyenne_1_20_j_corr]'
%% calcul distance sans obstacles formule erronée

distance_moyenne_1_2_s_o = (60*acos(sin(moy_1m_lat_dd_s_o)*sin(moy_2m_lat_dd_s_o)+cos(moy_1m_lat_dd_s_o)*cos(moy_2m_lat_dd_s_o)*cos(moy_2m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_5_s_o = (60*acos(sin(moy_1m_lat_dd_s_o)*sin(moy_5m_lat_dd_s_o)+cos(moy_1m_lat_dd_s_o)*cos(moy_5m_lat_dd_s_o)*cos(moy_5m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_10_s_o = (60*acos(sin(moy_1m_lat_dd_s_o)*sin(moy_10m_lat_dd_s_o)+cos(moy_1m_lat_dd_s_o)*cos(moy_10m_lat_dd_s_o)*cos(moy_10m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_15_s_o = (60*acos(sin(moy_1m_lat_dd_s_o)*sin(moy_15m_lat_dd_s_o)+cos(moy_1m_lat_dd_s_o)*cos(moy_15m_lat_dd_s_o)*cos(moy_15m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_20_s_o = (60*acos(sin(moy_1m_lat_dd_s_o)*sin(moy_20m_lat_dd_s_o)+cos(moy_1m_lat_dd_s_o)*cos(moy_20m_lat_dd_s_o)*cos(moy_20m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;

distance_moyenne_s_o = [distance_moyenne_1_2_s_o distance_moyenne_1_5_s_o distance_moyenne_1_10_s_o distance_moyenne_1_15_s_o distance_moyenne_1_20_s_o]'

%% calcul distance sans obstacles formule corrigée

distance_moyenne_1_2_s_o_corr = (60*acosd(sind(moy_1m_lat_dd_s_o)*sind(moy_2m_lat_dd_s_o)+cosd(moy_1m_lat_dd_s_o)*cosd(moy_2m_lat_dd_s_o)*cosd(moy_2m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_5_s_o_corr = (60*acosd(sind(moy_1m_lat_dd_s_o)*sind(moy_5m_lat_dd_s_o)+cosd(moy_1m_lat_dd_s_o)*cosd(moy_5m_lat_dd_s_o)*cosd(moy_5m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_10_s_o_corr = (60*acosd(sind(moy_1m_lat_dd_s_o)*sind(moy_10m_lat_dd_s_o)+cosd(moy_1m_lat_dd_s_o)*cosd(moy_10m_lat_dd_s_o)*cosd(moy_10m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_15_s_o_corr = (60*acosd(sind(moy_1m_lat_dd_s_o)*sind(moy_15m_lat_dd_s_o)+cosd(moy_1m_lat_dd_s_o)*cosd(moy_15m_lat_dd_s_o)*cosd(moy_15m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;
distance_moyenne_1_20_s_o_corr = (60*acosd(sind(moy_1m_lat_dd_s_o)*sind(moy_20m_lat_dd_s_o)+cosd(moy_1m_lat_dd_s_o)*cosd(moy_20m_lat_dd_s_o)*cosd(moy_20m_lon_dd_s_o-moy_1m_lon_dd_s_o))) * 1851.85;

distance_moyenne_s_o_corr = [distance_moyenne_1_2_s_o_corr distance_moyenne_1_5_s_o_corr distance_moyenne_1_10_s_o_corr distance_moyenne_1_15_s_o_corr distance_moyenne_1_20_s_o_corr]'






