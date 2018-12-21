import pycad
import numpy as np

def testArea():
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    np.set_printoptions(precision=2, suppress=True)

    a = np.array([[0,0], [1,1], [2,0]])
    b = np.array([[0,0,10], [10,0,1], [0,10,1]])
    ar,cent = pycad.polygonArea3D(b)
    
    print 'res', cent

    b = np.vstack((b, b[0]))
   
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(b[:,0], b[:,1], b[:,2], '-')
    ax.scatter(cent[0], cent[1], cent[2], 'ko')
    plt.show()

    '''
    
    plt.plot(a[:,0], a[:,1], '-')
    plt.plot(cent[0], cent[1], 'ko')
    plt.show()
    '''

def testUP():
    a = np.zeros((10,3)); a[4][2] = 10; a[6][1] = 5
    print a
    print uniqueRows(a)
    print uniquePoints(a)

def testCS():
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    m = pycad.StlMesh('member.STL')
    pts = m.crossSection([1300,460,1320], [1,0,0])

    
    print np.shape(m)
    
    st = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for xyz in pts:
        a, c = pycad.polygonArea3D(xyz, return_centroid=True)
        print 'area', a
        ax.plot(xyz[:,0], xyz[:,1], xyz[:,2])
        ax.scatter(c[0], c[1], c[2], 'ro')
    plt.show()

def testEA():
    m = pycad.StlMesh('member.STL')
    pycad.extrusionAxis(m)


if __name__ == '__main__':
    import cProfile, pstats

    from timeit import Timer

    setup = 'from pycad import stlMesh; m = stlMesh(\'member.STL\')'
    smt = 'm.crossSection([1300,460,1320], [1,0,0])'
    #t = Timer(smt, setup)
    #n = 1000
    #print t.timeit(n)/n

    #testArea()
    #testEA()
    testCS()
    '''
    cProfile.run('test()', 'testprof')
    p = pstats.Stats('testprof')
    p.sort_stats('cumulative').print_stats(10)
    '''


    '''
    pts = np.random.random((30,3))
    pts[:,0:2] *= 20
    pts = transform(pts, rotation_matrix(np.radians(45), [1,0,0]), onePad=True)

    #pts = radialSort(pts)
    '''
        
