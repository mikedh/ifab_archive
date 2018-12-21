
import json, os, shutil, sys, time
from zipfile import ZipFile
import robot, ifabModel


if __name__ == '__main__':

    #find local trunk location, assuming checkout from our SVN
    trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
    filename = trunk + 'workpiece-files/buttjoint.zip'
    

    extractDir = '.tmp'
    with ZipFile(filename, 'r') as archive:
        archive.extractall(extractDir)
    os.chdir(extractDir)
    try: j = json.load(open('metadata.json', 'r'))
    except: 'failed to load metadata.json, exiting'; sys.exit()

    #with input file loaded, loop through it
    try: 
        I = ifabModel.Inspector()
        for task in j['tasks']:
            for position in task['positioning']:
                model = ifabModel.stlModel(position['memberName'])
                #model.transform(position['transformationMatrix'])
                
                I.add(position['memberName'], model, position['ingress'])
            I.inspect(None)
                    
    finally:
        os.chdir('..')
        shutil.rmtree(extractDir)
        





