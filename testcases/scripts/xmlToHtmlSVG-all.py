#!/usr/bin/python
# author: Petros Faloutsos pfal@cse.yorku.ca
import sys, argparse
import glob
import xmlToHtmlSVG

def printHeader():
	print "<!DOCTYPE html>"
	print "<html>\n<head>"
	print "<meta charset=\"UTF-8\">"
	print "<title>TestCases</title>"
	print "</head>"
	print "<body>"

def printEnd():
	print "</body>"
	print "</html>"

def doAll(separateFiles=False):
	
	printHeader()
	fs = glob.glob("./*.xml")
	for f in fs:
		xmlToHtmlSVG.printHtml(f,noHeaders = True)
	printEnd()


if __name__ == "__main__":
	separateFiles = False
	parser = argparse.ArgumentParser(usage="Process all xml files in the current director into one html or separat files.")
	parser.add_argument("-s", "--separateFiles", action="store_true", help = "Not implemented yet!!!")
	args = parser.parse_args()
	if args.separateFiles:
		separateFiles = args.separateFiles
	doAll(separateFiles)