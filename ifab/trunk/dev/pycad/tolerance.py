import json
import numpy as np



Tangle = [-np.radians(5), np.radians(5)]
Tangle = None
Tdist = [-1,1]

VertA = [400]
VertB = [500]

itol = {'partVerticies' : [VertA, VertB], 'toleranceAngle' : Tangle, 'toleranceDistance' : Tdist}

tol = {'parts' : ['partA.stl', 'partB.stl'], 'toleranceList' : [itol, itol]}

t = [tol, tol, tol]
json.dump(t, open('tol.json', 'w'), sort_keys=True, indent=4)
