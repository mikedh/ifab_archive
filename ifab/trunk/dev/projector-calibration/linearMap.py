import numpy, math

import scipy.optimize as opt



filename = 'projector-values.txt'
data = numpy.loadtxt(filename, skiprows=1)
vecs = data[:,[4,5,0]]; vecs[:,[1,2]] *= -1;
pixs = data[:,[2,3]]


def squareInterp(v, ang=[.6812,.50747], res=[800,600]):
    h = 7
    if len(ang) == 3:
        h = ang[0]
        ang = numpy.array(ang)[1:]
    
    v = numpy.array(v).flatten()[0:3]
    v[2] += h

    cent = numpy.array(res)/2; 
    pxrad = numpy.array(res)/(ang)
    r = numpy.sqrt(numpy.dot(v,v))
    if r == 0: return 0
    if v[2] == 0: return 0
    v = v/r
    angs = numpy.array([numpy.arcsin(v[1]/v[2]), numpy.arcsin(v[0]/v[2])])
    #pxrad = pxrad[::-1]; angs = angs[::-1]
    res = (angs*pxrad) #+cent
    #print v, angs, pxrad, res, '\n'
    return res.flatten()[0:2]

#tmat is (2,3)
def matrixInterp(vec, tMat):
    if len(vec) <> 3: return 0
    if len(tMat) <> 13: return 0
    
    tMat = numpy.array(tMat)
    h = tMat[0]
    m = numpy.reshape(tMat[1:], ((3,4)))

    vec = (vec+[0,0,h])
    vec = vec/numpy.linalg.norm(vec)
    res = numpy.dot(m, numpy.append(vec,1))
    res = res / res[2]
    return res[0:2]
                    

#go through data set, calculate error with given tmat, and sum them
def mError(tMat, power=1.375, verb=False):
    serror = 0
    for i, v in enumerate(vecs):
        r =  matrixInterp(v, tMat) - pixs[i]
        s = numpy.diff((r, pixs[i]), axis=0)
        s = numpy.linalg.norm(s)**power
        if verb: print v, s, r,  pixs[i]
        if not math.isnan(s):
            serror += s
    return serror

def sError(angles, power=.8, verb=False):
    serror = 0
    for i, v in enumerate(vecs):
        r = squareInterp(v, angles)
        s = numpy.diff((r, pixs[i]), axis=0)
        s = numpy.linalg.norm(s)**power
        if verb: print v, s, r,  pixs[i]

        if not math.isnan(s):
            serror += s
    return serror

tM0 = numpy.append(0, numpy.identity(4)[0:3].flatten())
resM = opt.fmin_bfgs(mError, tM0)

#s0 = [0, .53, 0.8799]
#resS = opt.fmin_bfgs(sError, s0)




#es = sError(resS, power=1) #, verb=True)
em = mError(resM, power=1, verb=True)
#print 'square res', resS, 'square res error', es
print 'matrix res', resM, 'matrix res error', em



#print  'zero', squareInterp([-0,0,-400])
#print 'initial',  squareInterp(vecs[0], s0[1:]), pixs[0]
#print 'optimized',  squareInterp(vecs[0],res), pixs[0]
#print 'e0', e0


