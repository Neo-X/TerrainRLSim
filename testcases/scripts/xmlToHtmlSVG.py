#!/usr/bin/python
# author: Petros Faloutsos pfal@cse.yorku.ca

import xmltodict
import sys, argparse

from math import ceil
def printf(format, *args):
	sys.stdout.write(format % args)

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

def printStar(x,z,s=1.0):
	#points = [100,10 40,190 190,78 10,78 160,190]
	xpoints = [0.0, -2.0, 3.0, -3.0, 2.0]
	zpoints = [-3.0, 3.0, -0.7,  -0.7, 3.0]
	
	for i in range(0,5):
		xpoints[i] = s*float(xpoints[i]) + x
		zpoints[i] = s*float(zpoints[i]) + z
	printf(" <polygon points=\"")
	for i in range(0,4):
		printf("%f,%f ", xpoints[i],zpoints[i])
	printf("%f,%f\"\n",xpoints[4],zpoints[4])
	print 'style="fill:purple;stroke:purple;stroke-width:1" />'


def printHtml(file, noHeaders=False):
	with open(file) as fd:
		doc = xmltodict.parse(fd.read())

	# access the test case elements
	# They are stored as ordered dictionaries
	# header = doc['SteerBenchTestCase']['header']
	# store the world bounds
	wb = doc['SteerBenchTestCase']['header']['worldBounds']
	# cameras = doc['SteerBenchTestCase']['CameraViewsuggestedCameraView']

	obstacles = doc['SteerBenchTestCase'].get('obstacle')


	agents = doc['SteerBenchTestCase'].get('agent')
	agentRegions = doc['SteerBenchTestCase'].get('agentRegion')


	# print headers if needed
	if( not noHeaders): # printf header and context
		printHeader()

	# set the canvas to the world bounds
	wxmin = float(wb['xmin'])
	wxmax = float(wb['xmax'])
	wzmin = float(wb['zmin'])
	wzmax = float(wb['zmax'])

	
	# print the file name as a header
	printf("<h3> <a href=\"%s\"> %s </a></h3>\n", file,file)

	# scale factors for better visuals
	s = 1.0   # for the world and agents
	sStar = 0.1 #for the targets
	printf("<svg  width=\"%d\" height=\"%d\">\n ", ceil(s*(wxmax-wxmin)), ceil(s*(wxmax-wxmin)))
	
	# print the world bounds as a rect
	printf("<rect x=\"%f\" y = \"%f\" width=\"%f\" height=\"%f\"", 0,0, s*(wxmax-wxmin), s*(wzmax-wzmin)) 
	print 'style="fill:none;stroke-width:1;stroke:rgb(0,0,0)" />'

										   
	# print the obstacles
	if( obstacles != None ): #  if there are obstacles
		if not isinstance(obstacles, list):  # if only one obstacle is present obstacles is not a list
			# make it a list
			obstacles = [obstacles]
		for obj in obstacles:
			xmin =  s*(float(obj['xmin']) - wxmin)
			zmin =  s*(float(obj['zmin']) - wzmin)
			xmax =  s*(float(obj['xmax']) - wxmin)
			zmax =  s*(float(obj['zmax']) - wzmin)

			printf("<rect x=\"%f\" y = \"%f\" width=\"%f\" height=\"%f\"", xmin,zmin, xmax-xmin, zmax-zmin)
			print 'style="fill:rgb(0,0,255);stroke-width:1;stroke:rgb(0,0,0)" />'
	# print agents or agent region	

	if( agents != None ): #  if there are agents
		if not isinstance(agents,list): # if only one agent is present agents is not a list
			# make it a list
			agents = [agents]
		for agent in agents:
			x = s*(float(agent['initialConditions']['position']['x']) - wxmin)
			y = s*(float(agent['initialConditions']['position']['z']) - wzmin)
			r = s*float(agent['initialConditions']['radius'])
			printf("<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke=\"green\" stroke-width=\"1\" fill=\"yellow\" />\n", x,y,r)
		
			# print targets as stars
			targets = agent['goalSequence'].get('seekStaticTarget')
			if targets != None:
				if not isinstance(targets,list): # if only one target is present targets is not a list
					# make it a list
					targets = [targets]
				for target in targets:
					x = s*(float(target['targetLocation']['x']) - wxmin)
					z = s*(float(target['targetLocation']['z']) - wzmin)
					printStar(x,z,sStar) ;

	if( agentRegions != None):
		if not isinstance(agentRegions,list): # if only one agent is present agents is not a list
		# make it a list
			agentRegions = [agentRegions]
		for agentRegion in agentRegions:
			xmin = s*(float(agentRegion['regionBounds']['xmin']) - wxmin)
			zmin = s*(float(agentRegion['regionBounds']['zmin']) - wzmin)
			xmax = s*(float(agentRegion['regionBounds']['xmax']) - wxmin)
			zmax = s*(float(agentRegion['regionBounds']['zmax']) - wzmin)
			
			printf("<rect x=\"%f\" y = \"%f\" width=\"%f\" height=\"%f\"", xmin,zmin, xmax-xmin, zmax-zmin)
			print 'style="fill:rgb(255,0,0);stroke-width:1;stroke:rgb(0,0,0)" />'
			
			# print targets as stars
			targets = agentRegion['goalSequence'].get('seekStaticTarget')
			if targets != None:
				if not isinstance(targets,list): # if only one target is present targets is not a list
					# make it a list
					targets = [targets]
				print 'Targets ===== ', targets
				for target in targets:
					xt = target['targetLocation'].get('x')
					if xt != None :
						x = s*(float(xt) - wxmin)
						z = s*(float(target['targetLocation']['z']) - wzmin)
						printStar(x,z,sStar) ;


	print '</svg>'
	if( not noHeaders): # printf closing html statements
		printEnd() 

if __name__ == "__main__":
	inputfile = ''
	noHeaders = False
	parser = argparse.ArgumentParser(usage="Process an xml file into a html file.")
	parser.add_argument("inputfile", type=str, help = "The xml file to process.")
	parser.add_argument("-n", "--noheaders", type=str, help = "Do not output html main headers. By default it will.")
	
	args = parser.parse_args()
	inputfile = args.inputfile
	noHeaders = args.noheaders
	printHtml(inputfile, noHeaders)


