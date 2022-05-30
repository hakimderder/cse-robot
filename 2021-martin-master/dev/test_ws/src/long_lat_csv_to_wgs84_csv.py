#!/usr/bin/python
# -*- coding: UTF-8 -*-
import sys
from pandas import DataFrame, read_excel, read_csv
from degrees_long_lat_to_wgs84 import project as transform
import utm
import requests


debug = True
gnss_data_filename = "../../tests/mesures_gnss.csv"
wgs84_data_filename = "../../tests/mesures_wgs84.csv"
wgs84_utm_data_filename = "../../tests/mesures_wgs84_utm.csv"
mn95_data_filename = "../../tests/mesures_mn95.csv"

NAVREF_URI = "http://geodesy.geo.admin.ch/reframe/"



def mn95_to_wgs84(_x, _y):
    """
        Resquest conversion for mn95 to wgs84
    """
    res = None
    query = { "easting": _x, "northing": _y, "format": "json"}
    try:
        response = requests.get("{}lv95towgs84".format(NAVREF_URI), params=query)
        json_res = response.json()
        res = {"lon" :json_res["easting"], "lat": json_res["northing"]}
    except Exception as e:
        res = e
    return res


def navref_api_deg_to_mn95(_src_fname, _dst_fname, _add_day_time=True, _debug=False):
    """
        This function uses pyproj to calulate utm x and y
            zone calculated is :   32
            letter calculated is : T
    """
    func_name = "navref_api_deg_to_mn95"
    res = None

    try:
        # read data from csv
        df = read_csv(_src_fname, sep=";", encoding="utf8", dtype=str)
        if _debug:
            print("{}() : read csv ok, df : \n{}\n".format(func_name, df))

        x_y_list = []
        for idx_row in range(len(df)):
            long_data = float(df.iloc[idx_row]["Longitude mesuree"].replace(",", "."))
            lat_data = float(df.iloc[idx_row]["Latitude mesuree"].replace(",", "."))
            if long_data != float("nan") and lat_data != float("nan"):
                # make tuple from data
                query = { "easting": long_data, "northing": lat_data, "format": "json"}
                response = requests.get("{}wgs84tolv95".format(NAVREF_URI), params=query)
                json_res = response.json()
                if _debug:
                    print("{}() : est : {}\tnorth : {}".format(func_name, json_res["easting"], json_res["northing"]))
                if _add_day_time:
                    x_y_list.append([df.iloc[idx_row]["Jour"], df.iloc[idx_row]["Heure"], json_res["easting"], json_res["northing"]])
                else:
                    x_y_list.append([json_res["easting"], json_res["northing"]])
        
        if _debug:
            print("{}() : number of data calculated : {}".format(func_name, len(x_y_list)))

        # create csv
        if _add_day_time:
            _columns = ["Jour", "Heure", "mn95_e", "mn95_n"]
        else:
            _columns = ["mn95_e", "mn95_n"]
        res_df = DataFrame(x_y_list, columns=_columns)
        res_df.to_csv(_dst_fname, sep=";", index=False)
        
    except Exception as e:
        res = e
    return res


def pyproj_pkg_deg_to_utm(_src_fname, _dst_fname, _debug=False):
    """
        This function uses pyproj to calulate utm x and y
            zone calculated is :   32
            letter calculated is : T
    """
    func_name = "pyproj_pkg_deg_to_utm"
    res = None

    try:
        # read data from csv
        df = read_csv(_src_fname, sep=";", encoding="utf8", dtype=str)
        if _debug:
            print("{}() : read csv ok, df : \n{}\n".format(func_name, df))

        x_y_list = []
        for idx_row in range(len(df)):
            long_data = float(df.iloc[idx_row]["Longitude mesuree"].replace(",", "."))
            lat_data = float(df.iloc[idx_row]["Latitude mesuree"].replace(",", "."))
            if long_data != float("nan") and lat_data != float("nan"):
                # make tuple from data
                coord = (long_data, lat_data)
                z, l, x, y = transform(coord)  # transform accepts float !
                x_y_list.append([df.iloc[idx_row]["Jour"], df.iloc[idx_row]["Heure"], x, y])
        
        if _debug:
            print("{}() : number of data calculated : {}".format(func_name, len(x_y_list)))

        # create csv
        res_df = DataFrame(x_y_list, columns=["Jour", "Heure", "wgs84_x", "wgs84_y"])
        res_df.to_csv(_dst_fname, sep=";", index=False)
        
    except Exception as e:
        res = e
    return res


def utm_pkg_deg_to_utm(_src_fname, _dst_fname, _debug=False):
    """
        This function uses utm package to calculate y and x
    """
    func_name = "utm_pkg_deg_to_utm"
    res = None

    try:
        # read data from csv
        df = read_csv(_src_fname, sep=";", encoding="utf8", dtype=str)
        if _debug:
            print("{}() : read csv ok, df : \n{}\n".format(func_name, df))

        # make tuple from sheet p6170_variations_wgs84s
        x_y_list = []
        for idx_row in range(len(df)):
            long_data = float(df.iloc[idx_row]["Longitude mesuree"].replace(",", "."))
            lat_data = float(df.iloc[idx_row]["Latitude mesuree"].replace(",", "."))
            if long_data != float("nan") and lat_data != float("nan"):
                x, y, zone, letter = utm.from_latlon(lat_data, long_data)
                x_y_list.append([df.iloc[idx_row]["Jour"], df.iloc[idx_row]["Heure"], x, y])
        
        if _debug:
            print("{}() : number of data calculated : {}".format(func_name, len(x_y_list)))

        # create csv
        res_df = DataFrame(x_y_list, columns=["Jour", "Heure", "wgs84_x", "wgs84_y"])
        res_df.to_csv(_dst_fname, sep=";", index=False)
        
    except Exception as e:
        res = e
    return res


if __name__ == "__main__":
    three_args = len(sys.argv) == 3
    one_arg = len(sys.argv) == 1
    if three_args and sys.argv[1][:-1].isdigit() and sys.argv[2][:-1].isdigit() and sys.argv[1].endswith("E") and sys.argv[2].endswith("N"):
        res_dict = mn95_to_wgs84(_x=int(sys.argv[1][:-1]), _y=int(sys.argv[2][:-1]))

        if type(res_dict) is dict:
            print("Longitude [E] : {}\t Latitude [N] : {}".format(res_dict["lon"], res_dict["lat"]))
        else:
            print("Something went terribly wrong : {}".format(res_dict))
    elif (three_args and sys.argv[1].endswith(".csv") and sys.argv[2]) or one_arg:
        source_filename = "{}.csv".format(sys.argv[1] if three_args else gnss_data_filename)
        destination_filename = "{}.csv".format(sys.argv[2] if three_args else mn95_data_filename)

        # Convert files
        res_ok = navref_api_deg_to_mn95(_src_fname=sys.argv[1], _dst_fname=sys.argv[2], _add_day_time=False, _debug=True)

        if res_ok is None:
            print("Process terminated correctly")
        else:
            print("Something went terribly wrong : {}".format(res_ok))
    else:
        print("Wrong usage\nExample args :\n\t25406222E 181302N\n\tmy_src_file.csv my_dst_file.csv")

    # The end

