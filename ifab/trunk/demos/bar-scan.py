import robot, transformations, jackDistance
import numpy, time, sys

import Gnuplot

#Reads position sensor, switches tool to welder, and then moves into position
ron = '-r' in sys.argv
v = '-v' in sys.argv
plot = '-p' in sys.argv


if __name__ == '__main__':

    try: 
        if ron: 
            R = robot.Robot(verbose=v, zeroJoints = True)
            #L = robot.Logger(verbose=v)
        if plot: g = Gnuplot.Gnuplot()


        ddir = '../data/'

        tq = [0,0,1,0]  

        alltp = numpy.loadtxt(ddir + 'zig-spect.txt')
        firstPos = alltp[0:int(len(alltp)/2)]; fp = []
        secPos = alltp[(int(len(alltp)/2)+2):len(alltp)]; sp = []
        for i in firstPos: fp.append([i,tq])
        for i in secPos: sp.append([i,tq])

        if ron: 
            R.setToolFile(ddir + 'tool.object.displacement')
            R.setZone('z10')

            R.setBuffer(fp, gotoFirst=True)
            time.sleep(.5)
            print "start record"
            L.start()
            R.executeBuffer()
            time.sleep(.5)
            cloud = L.stop(sequence=True)
            print "end record"

            if cloud <> None: 
                print 'recorded data count', len(cloud)
                if plot: 
                    d = cloud
                    g.splot(d)

                numpy.savetxt((ddir + 'scans/cloud-' + time.strftime('%d-%H%M') + '-0.txt'), cloud)


            R.setBuffer(sp, gotoFirst=True)
            time.sleep(.5)
            print "start record"
            L.start()
            R.executeBuffer()
            time.sleep(.5)
            cloud1 = L.stop(sequence=True)
            print "end record"

            if cloud1 <> None: 
                if plot:
                    d = numpy.vstack((cloud, cloud1))
                    g.splot(d)

                print 'recorded data count', len(cloud)
                numpy.savetxt((ddir + 'scans/cloud-' + time.strftime('%d-%H%M') + '-1.txt'), cloud)

    finally: 
        L.close()
        if ron: del R
        quit = raw_input('exit?')
