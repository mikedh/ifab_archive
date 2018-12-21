# a stacked bar plot with errorbars
import numpy, os, cPickle
import matplotlib.pyplot as plt


times = {'gross-motion': [], 'placement':[], 'inspection': [], 'welding': [], 'computation': []}
trunk = os.getcwd()[0:(6+os.getcwd().rfind('trunk'))]
scand = trunk + 'data/scans/'; lmax = 0
for fn in reversed(os.listdir(scand)):
    if fn.find('times') <> -1:
        fn = 'times-01-1853.pickle'
        try: d = cPickle.load(open((scand+fn), 'r'))
        except: continue
        print 'loaded', fn
        for name in times.keys():
            times[name] = numpy.append(times[name], d[name])
            l = numpy.shape(times[name])[0]
            if l > lmax: lmax = l
        break

del times['gross-motion']
#zero pad
ta = numpy.zeros((len(times.keys()), lmax))
for i, name in enumerate(times.keys()):
    l = numpy.shape(times[name])[0]
    if l < lmax:  
        ta[i] = numpy.append(times[name], numpy.zeros((1,lmax-l)))
    else: 
        ta[i] = times[name]
ta = numpy.rot90(ta)
print ta



N = numpy.shape(times.keys())[0]
ind = numpy.arange(N)    # the x locations for the groups
width = 0.35       # the width of the bars: can also be len(x) sequence

plt.bar(ind, ta[0],   width, color='r')
for i in numpy.array(range(lmax))[1:]:
    print 'ind', ind
    plt.bar(ind, ta[i],   width, color='r', bottom=ta[i-1])
    #plt.bar(ind, womenMeans, width, color='r',bottom=menMeans)

plt.ylabel('Time(s)')
plt.title('iFAB excution time on \'hood-flat\': total ' + format(numpy.sum(ta), '01.2f') + 's')
plt.xticks(ind+width/2., times.keys() )
plt.yticks(numpy.arange(0,45,5))
#plt.legend( (p1[0], p2[0]), ('Men', 'Women') )

plt.show()

