# -*- coding: utf-8 -*-

# Copyright 2017 - 2017 Keysight Technologies, Inc , Keysight Confidential

def main():
	import os, sys
	varDictionary={}
	exprDictionary={}
	ADS_HPEESOF_DIR=r"C:\Program Files\Keysight\ADS2017_Update1"
	libName=r"ANT_PIFA_1_Design_lib"
	substName=r"substrate"
	workspacePath=r"C:\Users\aurelien_berthelot\Desktop\fichiers_TB\TB\TB\electronique\Module_RTK\antenne_rf\Labo_07_wrk_For_Students\ant_868MHz_v1"
	libdefs=r"lib.defs"
	envEMPROHOME=r"C:\Program Files\Keysight\ADS2017_Update1\fem\2017.10/Win32_64/bin"
	verbose=False
	sys.path.append(os.path.join(envEMPROHOME))
	import empro.toolkit.via_designer as via_designer

	via_designer.mainGui.main(varDictionary, exprDictionary, ADS_HPEESOF_DIR, libName, substName, workspacePath=workspacePath, libdefs=libdefs, verbose=verbose)

if __name__=="__main__":
	main()
