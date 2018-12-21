def ft(t, a=1.0, vec=[.1,0], p0=[0,0]):
    p = [(a * numpy.cos(t) + vec[0]*t), (a * numpy.sin(t) + vec[1]*t)]
    if vec[0] == 0: at = numpy.pi
    else: at = numpy.arctan((vec[1]/vec[0])) + numpy.pi
    pset = -1*numpy.array([(a * numpy.cos(at) + vec[0]*at), (a * numpy.sin(at) + vec[1]*at)])
    return p + pset + p0

def dist(a,b):
    return numpy.sqrt(numpy.sum((numpy.array(a)+b)**2))

def spiral(p0=[1500,0], vec=[1,0], d=600.0, a=50, cnt=9, tseg=(numpy.pi/6), z=-100, quats = [0,0,1,0]):
    t0 = 0; t = t0; tf = numpy.pi*cnt*2+tseg; path = [];
    vec = (vec/numpy.linalg.norm(vec)) * ((d-2*a)/tf)
    while t <= tf:
        v = numpy.append(ft(t, a, vec, p0), z)
        path.append([v,quats])
        t += tseg
    path = numpy.array(path);     
    return path
