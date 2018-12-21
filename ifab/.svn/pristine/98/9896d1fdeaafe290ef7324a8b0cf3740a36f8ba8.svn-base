import argparse, numpy



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-filename', '-f' , dest="filename" ,required=True)
    parser.add_argument('-localdata', '-l' , dest="datakey" )
    parser.add_argument('-inspectonly', '-i' , action='store_true')
    parser.add_argument('-weld', '-w' , action='store_true')
    cli = parser.parse_args()
    print cli.datakey
    try: 
        d = numpy.loadtxt(open(cli.filename, 'r'))
        print d
    except:
        print 'file load failed'
    if cli.inspectonly:
        print cli.datakey
    else:
        print 'no'
