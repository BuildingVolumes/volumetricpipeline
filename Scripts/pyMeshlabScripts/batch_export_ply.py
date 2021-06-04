import pymeshlab
import os
import sys, getopt



def main(argv):
   inputDir = ''
   outputDir = ''
   oSpecified = False
   recon = False
   convert = False
   extension = ""
   try:
      opts, args = getopt.getopt(argv,"hrd:o:c:",["dir=","out=","convert="])
   except getopt.GetoptError:
      print('test.py -d <data dir> -o <optional output dir> -r')
      sys.exit(2)
   if( len(opts) == 0):
      	print('test.py -d <data dir> -o <optional output dir> -r (reconstruct) -c (extension to convert to)')
      	sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print('test.py -d <data dir> -o <optional output dir> -r (reconstruct) -c (extension to convert to, i.e. .ply, .obj)')
         sys.exit()
      elif opt in ("-d", "--dir"):
         inputDir = arg
      elif opt in ("-o", "--out"):
        outputDir = arg
        oSpecified = True
      elif opt in ("-r","--recon"):
         recon = True
      elif opt in ("-c","--convert"):
         convert = True
         extension = arg

   if(oSpecified == False):
      outputDir = inputDir

   print('Input DIR is ' +inputDir)
   print('Output DIR is '+ outputDir)
   for filename in sorted(os.listdir(inputDir)):
      if filename.endswith(".ply") :
         fname = os.path.join(inputDir, filename)
         ofname = os.path.join(outputDir,filename)+extension
         print('processing:'+fname + " --> "+ofname)
         ms = pymeshlab.MeshSet()
         ms.load_new_mesh(fname)
            
         if recon == True :
            print("recon:")  
            ms.compute_normals_for_point_sets()
            #ms.per_vertex_normal_function()
            ms.surface_reconstruction_screened_poisson()
 
         ms.save_current_mesh(ofname)
      else:
         continue







if __name__ == "__main__":
   main(sys.argv[1:])


