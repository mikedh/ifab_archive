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



s = numpy.array([["SF_Bottom_001-1","SF_Pillar02_004-1"],["SF_Pillar01a_003-1","SF_Bottom_001-1"],["SF_Top_002-1","SF_Pillar02_004-1"],["SF_Pillar01b_007-1","SF_Pillar01a_003-1"],["SF_Pillar01b_007-1","SF_Top_002-1"],["SF_Bottom_001-1","SF_Front02_009-1"],["SF_Front02_009-1","SF_Front01_008-1"],["SF_Front02_009-1","SF_Front03_010-1"],["SF_Front03_010-1","SF_Pillar01a_003-1"],["SF_Pillar03a_005-1","SF_Top_002-1"],["SF_Pillar03b_006-1","SF_Pillar03a_005-1"],["SF_Pillar03b_006-1","SF_Bottom_001-1"],["SF_WingHorizontal_011-1","SF_Pillar03a_005-1"],["SF_WingHyp_012-1","SF_Top_002-1"],["SF_WingHyp_012-1","SF_WingHorizontal_011-1"]])

s = numpy.unique(s.flatten())
sM = short2long(s, F.keys())
tmat = transformations.rotation_matrix(numpy.radians(90), [1,0,0]).tolist()
tmat[0][3] = 1150
tmat[1][3] = 600
#tmat = numpy.dot(transformations.rotation_matrix(numpy.radians(90), [0,0,1]), tmat).tolist()

L = len(j['tasks'][0]['positioning'])

for i in xrange(6): #len(sM)):
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

j['tasks'][0]['positioning'].append(deepcopy(j['tasks'][0]['positioning'][0]))
tmat1 = transformations.rotation_matrix(numpy.radians(90), [1,0,0]).tolist()
tmat1 = numpy.dot(transformations.rotation_matrix(numpy.radians(-90), [0,0,1]), tmat1).tolist()
tmat1[0][3] = 1150+1200
tmat1[1][3] = 600+1700

j['tasks'][0]['transformationMatrix'] = tmat1
for i, s in enumerate(numpy.append(0,range(6,12))):
    print i,s
    j['tasks'][0]['positioning'][i]['memberName'] = sM[s]
    j['tasks'][0]['positioning'][i]['memberInt'] = F[(str(sM[s]) + '.STL')] 
json.dump(j, open('metadata-1.json', 'w'), sort_keys=False, indent=4)
