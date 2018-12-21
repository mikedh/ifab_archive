import json, numpy
import transformations
from copy import deepcopy
j = json.load(open('mdold.json', 'r'))
F = json.load(open('fileNames.txt', 'r'))


def short2long(shortN, longN):
    res = []
    for i, s in enumerate(shortN):
        for L in longN:
            if s in L: 
                res.append(L[0:-4])
                break
    return res





s = numpy.array([["ISF_Bottom_013-1","ISF_MidPillar_015-1"],["ISF_MidPillar_015-1","ISF_Horizontal_014-1"],["ISF_Bottom_013-1","ISF_RearPillar_016-1"],["ISF_Horizontal_014-1","ISF_RearPillar_016-1"],["ISF_Bottom_013-1","SF_Front02_009-1"],["SF_Front01_008-1","SF_Front02_009-1"],["SF_Front01_008-1","SF_Front03_010-1"],["ISF_Top01_017-1","ISF_Horizontal_014-1"],["ISF_Top02_018-2","ISF_Top01_017-1"],["ISF_Top02_018-2","SF_Front03_010-1"],["ISF_TopRF_019-1","SF_Front03_010-1"],["ISF_TopRF_019-1","ISF_Top02_018-2"]])



s = numpy.unique(s.flatten())
sM = short2long(s, F.keys())
tmat = transformations.rotation_matrix(numpy.radians(90), [1,0,0]).tolist()
tmat[0][3] = 1150
tmat[1][3] = 600
#tmat = numpy.dot(transformations.rotation_matrix(numpy.radians(90), [0,0,1]), tmat).tolist()

L = len(j['tasks'][0]['positioning'])

for i in xrange(len(sM)):
    if i >= L:
        j['tasks'][0]['positioning'].append(deepcopy(j['tasks'][0]['positioning'][0]))
    j['tasks'][0]['positioning'][i]['transformationMatrix'] = tmat
    j['tasks'][0]['positioning'][i]['memberName'] = sM[i]
    print j['tasks'][0]['positioning'][i]['memberName'] 
    j['tasks'][0]['positioning'][i]['memberInt'] = F[(str(sM[i]) + '.STL')] 
    
    try: 
        del j['tasks'][0]['positioning'][i]['transformationMatrix'] 
        del j['tasks'][0]['positioning'][i]['ingress'] 
    except: pass #print j['tasks'][0]['positioning'][i].keys()
    

    
j['tasks'][0]['transformationMatrix'] = tmat
json.dump(j, open('metadata-0.json', 'w'), sort_keys=False, indent=4)
