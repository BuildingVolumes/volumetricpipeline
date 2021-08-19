import os
from PIL import Image #https://pillow.readthedocs.io/en/stable/
import argparse

from os import listdir, getcwd, system, replace, walk
from os.path import isfile, isdir, join
import re


parser = argparse.ArgumentParser(description='Convert Depth tiff to PNG ')
parser.add_argument('inputDir', type=str, help='Directory of Images to Process')
parser.add_argument('--delete', required=False,action="store_true", help='delete original tiff files')

args = parser.parse_args()
inputDir = args.inputDir
if args.delete:
	print("DELETE")

print(inputDir);

#exit(1)
for currentDir, dirnames, filenames in walk(inputDir):
	print (f'Processing directory: {currentDir}')
	firstFile = True;
	for filename in filenames:
		fileMatch = re.match(r"(?P<base>.*)\.(?P<ext>tiff)$", filename)
	
		if fileMatch:
			base = fileMatch.group('base')
			ext = fileMatch.group('ext')
			fn = join(currentDir, filename)
			outputFN = join(currentDir,f'{base}.png')
			print(outputFN)
			fullFilename= join(currentDir, filename);
			print(f'Converting: {fullFilename}')
			print(f'........to: {outputFN}')
			tiff = Image.open(fullFilename)
			tiff.save(outputFN, 'png')
			if args.delete:
				print(f'..Deleting: {fullFilename}')
				os.remove(fullFilename)

	
 