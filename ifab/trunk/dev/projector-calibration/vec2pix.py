import numpy
import matplotlib.pyplot as plt



#load a data file with the following columns:
#nominalZ    index#	pixelX	pixelY	tableX	tableY

#calculates the corrected Z value based on the data, and can plot the spread in a histogram

class ProjectorTraining:
    def __init__(self, filename='data-mabaran.txt', Zcorrection=False):
        #load the data file, skip the header
        self.data = numpy.loadtxt(filename, skiprows=1)
        self.difh = []; self.vecs = []; self.pixs = []
        #loop through the corner indexes
        for i in xrange(int(numpy.max(self.data[:,1]))+1):
            cpoint = self.data[self.data[:,1]==i]
            print 'cpoint: \n', cpoint
            for j in xrange(numpy.shape(cpoint)[0]-1):
                dh = cpoint[j+1,0] - cpoint[j,0]
                print 'dh: ', dh
                dd = abs(abs(norm(cpoint[j+1, 4:6])) - abs(norm(cpoint[j, 4:6])))
                print 'distance: ', dd
                tth = (dd/dh)
                print 'tanth: ', tth
                if (tth <> 0): self.difh = numpy.append(self.difh, [(numpy.array(norm(cpoint[j, 4:6]))/tth) - cpoint[j,0]])     
        
        if Zcorrection: 
            self.zcorr = numpy.mean(self.difh)
            self.data[:,0] += self.zcorr
            print self.zcorr
 
        pixlocation = numpy.array([False, False, True, True, False, False], dtype=bool)
        for i in xrange(numpy.shape(self.data)[0]):
            vec = [self.data[i][4], self.data[i][5], -self.data[i][0]]

            if (numpy.shape(self.vecs)[0] == 0): self.vecs = vec
            else: self.vecs = numpy.vstack((self.vecs, vec))

            if (numpy.shape(self.pixs)[0] == 0): self.pixs = self.data[i][pixlocation]
            else: self.pixs = numpy.vstack((self.pixs, self.data[i][pixlocation]))

        
    #calclates angle between all data vectors and input vector, and does a 1/angle weighted average
    def vec2pix(self, vector, PTOL = .001, n=3):
        weights = numpy.zeros((numpy.shape(self.vecs)[0],1))
        
        for i in xrange(numpy.shape(self.vecs)[0]):
            vang = angle(vector, self.vecs[i])
            if vang < PTOL: return self.pixs[i]
            else:
                weights[i] = vang ** -n
        weights = weights/numpy.sum(weights)
        weighted = self.pixs*weights
        res = [numpy.sum(weighted[:,0]), numpy.sum(weighted[:,1])]
       
        return res
                


    def plotZhist(self):
        #print self.difh
        plt.hist(self.difh, 10)
        plt.show()
        




def angle(a,b): return numpy.abs(numpy.arcsin(norm(numpy.cross(a,b))/(norm(a)*norm(b))))

def norm(v):
    vec = numpy.array(v).flatten()
    return numpy.sqrt(numpy.dot(vec,vec))

def unitv(v):
    v = numpy.array(v).flatten()[0:3]
    n = norm(v)
    if (n<>0): return v/norm(v)
    else: return v

def parallel(v1,v2, PTOL=1e-2):
    if (abs(norm(v1)-norm(v2)) < PTOL): return True
    else: return False


if __name__ == '__main__':

    a = ProjectorTraining(filename='projector-values.txt')
    #a = ProjectorTraining(filename='data-mabaran.txt')
    #print a.difh
    a.plotZhist()
    #print '-45,0 ', a.vec2pix([-45,0,-250])


    '''

    print '0,60 ', a.vec2pix([0,60,-200])
    print '0,90 ', a.vec2pix([0,90,-200])
    print '0,120 ', a.vec2pix([0,120,-200])

    print '30,0 ', a.vec2pix([30,0, -200])
    print '60,0 ', a.vec2pix([60, 0,-200])
    print '90,0 ', a.vec2pix([90, 0, -200])
    print '120,0 ', a.vec2pix([120,0,-200])






    xpts = []; ypts = [];
    xext = 30; yext = xext; 
    res = 12
    zc = -500
    for i in xrange(-xext, xext, res):
        for j in xrange(-yext, yext, res):
            pt = a.vec2pix([i, j, zc], n=1.5)
            xpts = numpy.append(xpts,pt[0])
            ypts = numpy.append(ypts,pt[1])


    plt.plot(xpts,ypts, 'ro')
    plt.show()

    '''
