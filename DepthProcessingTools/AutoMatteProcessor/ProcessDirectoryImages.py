# example usage:
# python .\ProcessDirectoryImages.py INPUT_DIR MODELFILE
# python .\ProcessDirectoryImages.py C:\Users\hogue\Desktop\DATA\july15-bg_1\july15-bg_1\ --model .\Models\torchscript_resnet50_fp32.pth       
#
# Requirements: must have torchvision with cuda, model must be torchscript type
# this will process all .jpg files in a directory structure and will process them recurcively
# so for livescan3d Take output, just call on inputDir=mainTakeDirectory/ and it will go through each client
# 
# Assumes that the FIRST image in each dir is the background reference to use
# TODO: can probably override this with a commandline optional arg later
# model tested with : https://drive.google.com/uc?id=1-t9SO--H4WmP7wUl1tVNNeDkq47hjbv4 
# check here for future models: https://drive.google.com/drive/folders/1cbetlrKREitIgjnIikG1HdM4x72FtgBh

import sys

MIN_PYTHON = (3, 7)
assert sys.version_info >= MIN_PYTHON, f"requires Python {'.'.join([str(n) for n in MIN_PYTHON])} or newer"

from os import listdir, getcwd, system, replace, walk
from os.path import isfile, isdir, join
import re
from shutil import rmtree
from subprocess import Popen

import torch
from torchvision.transforms.functional import to_tensor, to_pil_image
from PIL import Image

import argparse

parser = argparse.ArgumentParser(description='BackgroundMattingV2 wrapper')
parser.add_argument('inputDir', type=str, help='Directory of Images to Process')
parser.add_argument('--model', type=str, help='the resnet model')
parser.add_argument("--bgDir", type=str, help='optional bg reference dir (same structure as inputDir) ')

args = parser.parse_args()
inputDir = args.inputDir
model = ".\\Models\\torchscript_resnet50_fp32.pth"
if args.model:
	model = args.model

BGProvided = False
if args.bgDir :
    bgRefDir = args.bgDir
    BGProvided = True
    print(f'BGProvided: {bgRefDir}')

print(inputDir)
print(model)

theModel = torch.jit.load(model).cuda().eval()

for currentDir, dirnames, filenames in walk(inputDir):
    print (f'Processing directory: {currentDir}')
    firstFile = True;
    for filename in filenames:
        fileMatch = re.match(r"(?P<base>.*)\.(?P<ext>jpg)$", filename)
        
        if fileMatch:
            base = fileMatch.group('base')
            ext = fileMatch.group('ext')
            fn = join(currentDir, filename)
            outputFN = join(currentDir,f'{base}.matte.png')
            subdir=currentDir.replace(inputDir,'').replace('\\','')
            print(f'subdir={subdir}')
            if firstFile:
                firstFile = False
                if not BGProvided:
                    bgRef = fn
                else:
                    bgRef = join(bgRefDir,subdir,f'{base}.{ext}')
             
            print(f'BG:   {bgRef}')
            print(f'FILE: {fn}')
            print(f'OUT:  {outputFN}')
            src = Image.open(fn)
            bgr = Image.open(bgRef)
            src = to_tensor(src).cuda().unsqueeze(0)
            bgr = to_tensor(bgr).cuda().unsqueeze(0)
            
            if src.size(2) <= 2048 and src.size(3) <= 2048:
              theModel.backbone_scale = 1/4
              theModel.refine_sample_pixels = 80_000
            else:
              theModel.backbone_scale = 1/8
              theModel.refine_sample_pixels = 320_000
             
            pha, fgr = theModel(src, bgr)[:2]
            to_pil_image(pha[0].cpu()).save(outputFN)
