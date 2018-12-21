#import scipy.optimize as opt
import numpy, json

from copy import deepcopy
#inventory is list like:
#   [[qty, length]]

#cut list is 
#    [ length, length, ...]

#solution is:
#    [index, index, ...]


#scrap is defined as the length remaining from pieces which have had cuts taken from them

#not really using this function, it was from earlier
def scrap(x, *args):
    cuts = args[0]
    inventory = args[1]
    cutouts = numpy.zeros(len(inventory))
    for whichcut, whichinventory in enumerate(x):
        cutouts[int(numpy.round(whichinventory*(len(inventory)-1)))] += cuts[whichcut]
    leftover = (inventory - cutouts)*(cutouts > 0)
    if numpy.any(leftover < 0): 
        ose = ( numpy.sum(numpy.abs((leftover < 0)*leftover))**2)
        print 'stock is oversubscribed, reported error', ose
        return ose
    return numpy.sum(leftover)**2


def seqError(x, cuts, inventory, verbose=False, pindex=False):
    scrap = 0; cpiece = 0; pleft = inventory[0]
    plist = numpy.zeros(len(cuts))
    for i, cutnum in enumerate(x):
        if cuts[cutnum] < pleft:
            pleft -= cuts[cutnum]
        else: 
            scrap += pleft; cpiece +=1
            try: 
                pleft = inventory[cpiece] - cuts[cutnum]
            except: print cpiece, cutnum
        if pindex: plist[i] = cpiece
    if pindex: return plist
    return scrap + pleft

def randomSeq(cuts, inventory, tries=1000):
    clen = len(cuts)
    ebest = 1e9; sbest = None
    for i in xrange(tries):
        x = numpy.random.permutation(clen)
        e = seqError(x, cuts, inventory)
        if e < ebest: 
            print 'better solution than', ebest, 'is', e#, x,
            ebest = e
            sbest = x
    
    return sbest


def cutLength(exactCut, minScrap=(25.4*1), increment=(25.4*2)):
    r = exactCut + minScrap
    r += increment - (r % increment)
    return r


if __name__ == '__main__':
    invQ = [[24, 144]]; 
    maxlen = 5; cutcount = 25; inv = []
    
    for i in invQ: inv = numpy.append(inv, (i[0]*[i[1]]))
    #cuts = (12*25.4*maxlen*numpy.random.rand(cutcount))
        

    d = json.load(open('roughCuts.txt', 'r'))
    
    cuts = numpy.array(d.values())
    for i in xrange(len(cuts)): cuts[i] = cutLength(cuts[i], 0, 12)
    cuts = numpy.int_(cuts)
    u = numpy.unique(cuts)
    
    k = numpy.array(d.keys())
    r = []

    for i in u:
        r.append(str(i) + '.0 (inches) qty: ' + str(numpy.sum([cuts == i])) + ' serials: ' + str((k[cuts == i])))
        
    
    json.dump(r, open('qtyCuts.txt', 'w'), sort_keys=False, indent=4)
    #seq = randomSeq(cuts, inv, 90000)
    #pnum = seqError(seq, cuts, inv, pindex=True)
    
    
    '''

    cuts = cuts[[seq]]
    
    

    
    for i in xrange(len(cuts)):
        r.append( 'serial: ' + str(k[i]) + '    cut length (inches): ' +  str(numpy.round(cuts[i])).zfill(4) + '    from inventory #: ' + str(int(pnum[i])))
    
    '''
