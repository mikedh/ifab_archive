'''
Copyright (c) 2012, Carnegie Mellon University

Developed with the sponsorship of the Defense Advanced Research Agency

Permission is hereby granted, free of charge, to any person obtaining a copy of this data, including any software or models in source or binary form, specifications, algorithms, and documentation (collectively "the Data”), to deal in the Data without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Data, and to permit persons to whom the Data is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Data.

THE DATA IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS, SPONSORS, DEVELOPERS, CONTRIBUTORS, OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DATA OR THE USE OR OTHER DEALINGS IN THE DATA

Michael Dawson-Haggerty
CMU Robotics Institute
For Dr. David Bourne/iFAB

ifab-main.py: main level iFAB welding script
Used to load a generic iFAB welding format zip file
usage: python ifab-main.py ifab-file.zip (-w: enables welding vs simulated welding)

Loops through a JSON structure and reads/manipulates 3D models based on the information contained.

Physical placement of a member is guided with a laser projection system, scanned resulting in a point cloud, and then that very accurate data is used to align the weld, so that parts which are misaligned on an absolute scale, but correct on a relative scale can succeed. We can also use this data to calculate if the relative positions are within tolerance before welding. 
'''


import json, os, shutil, sys, time, pickle, argparse,numpy,pdb
from zipfile import ZipFile
import matplotlib.pyplot as plt
from datetime import datetime
import robot, ifabModel
import ifabPCL as pcl

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='executes an iFAB welding plan. Example usage for an alignment test using dataset *24-1849.txt would be:    python ifab-main.py -f hood-flat.zip -l 24-1849')
    parser.add_argument('-filename', '-f' , dest="filename" , required=True, type=str, help='filename in local directory or trunk/workpieces/ for zip archive with iFAB structure')
    parser.add_argument('-local', '-l' , dest="cloudkey", help='runs main without interfacing with any hardware functions, for alignment testing')
    parser.add_argument('-inspectonly', '-i' , action='store_true', help='inspect, save data files, but don\'t project or calculate weld')
    parser.add_argument('-zigzag', '-z' , action='store_true', help='use a zig-zag individual member inspection')
    parser.add_argument('-weld', '-w' , action='store_true', help='if an aligned weld is calculated, execute real welds')
    parser.add_argument('-skipprojection', '-s' , action='store_true', help='skip projection step, but calculate and weld')
    cli = parser.parse_args(); local = cli.cloudkey <> None
    
    
    trunk = os.path.abspath(os.getcwd()[0:(6+os.getcwd().rfind('trunk'))])

    wfname = os.path.join(trunk, 'workpieces', str(cli.filename).strip())
    if os.path.exists(wfname): filename = wfname
    elif os.path.exists(str(cli.filename).strip()): filename = str(cli.filename).strip()
    else: print 'file', cli.filename, 'not found'; sys.exit()

    extractDir = '.tmp'
    with ZipFile(filename, 'r') as archive:
        archive.extractall(extractDir)
    os.chdir(extractDir)
    try:
        j = json.load(open('metadata.json', 'r'))
        print 'loaded', filename
    except: print 'failed to load metadata.json, exiting'; sys.exit()
    

    #with input file loaded, loop through it
    try:
        if not local: R = robot.Robot(zeroJoints=True)
        else: R = None
    
        I = ifabModel.Inspector()
        
        for task in j['tasks']:
            
            for position in task['positioning']:
                try: 
                    model = ifabModel.stlModel(position['memberName'])
                    model.transform(position['transformationMatrix'])
                    print numpy.shape(model.points)
                    if (not local) and (not cli.skipprojection):
                        R.setJoints(model.projectIngress())
                        model.projectFixtures(R)
                        print 'current robot joints', R.getJoints()
                        #model.project(R)
        
                    I.add(position['memberName'], model, position['ingress'])
                except: print 'err:', position
            if local: I.loadClouds(cli.cloudkey)
            else: 
                if cli.zigzag: I.inspectZig(R)
                else: I.inspectGlobal(R)
                print 'exited run segments'

            if not cli.inspectonly:
                I.templateMatch()
                I.dump()
                I.runSegments(R);
                
                welds = I.calcWelds()
                print welds

                for w in welds:
                    ingress = ifabModel.weldIngress(w)
                    print ingress
                    if not local: 
                        R.setSpeed((250,100))
                        R.setToolFile(os.path.join(trunk, 'data', 'tool.object.welder'))
                        R.setJoints(ingress)
                        R.setBufferMod(w)

                        if cli.weld: R.weldBuffer()
                        else: 
                            R.executeBuffer()
                            g = raw_input('to weld, type weld: ')
                            if g == 'weld':
                                R.setJoints(ingress)
                                R.weldBuffer()

                        R.setJoints(ingress)
                if not local:
                    R.setSpeed()
                    R.setJoints()
               


    finally:
        del R
        os.chdir('..')
        shutil.rmtree(extractDir)
        
