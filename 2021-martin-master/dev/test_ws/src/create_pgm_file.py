#!/usr/bin/python

import numpy as np
import sys



def create_pbm(_width, _height, _val=None, _filename="test.pgm", _debug=False):
    """
        source : https://stackoverflow.com/questions/40082165/python-create-pgm-file
    """
    func_name = "create_pbm_2"
    p_num = width * height
    arr = np.random.randint(0,255,p_num) if _val is None else [_val] * p_num

    # open file for writing 
    fout=open(_filename, 'wb')

    # define PGM Header
    pgmHeader = 'P5' + ' ' + str(width) + ' ' + str(height) + ' ' + str(255) +  '\n'

    pgmHeader_byte = bytearray(pgmHeader,'utf-8')

    # write the header to the file
    fout.write(pgmHeader_byte)

    # write the data to the file 
    img = np.reshape(arr,(height,width))

    for j in range(height):
        bnd = list(img[j,:])
        if _debug:
            print("{}() : bnd : {}".format(func_name, bnd))
        bnd_byte = bytearray(bnd)     
        fout.write(bnd_byte)

    fout.close()



if __name__ == '__main__':
    
    nb_param = len(sys.argv)
    if nb_param < 3:
        print("At least width and height should be given as parameters : create_pgm_file.py 100 100")
        quit()

    # define the width  (columns) and height (rows), content and filename of the image
    width = int(sys.argv[1])
    height = int(sys.argv[2])
    value = int(sys.argv[3]) if nb_param >= 4 else None
    filename = sys.argv[4] if nb_param == 5 else "empty_map.pgm"
    
    create_pbm(_width=width, _height=height, _val=value, _filename=filename, _debug=False)
    
