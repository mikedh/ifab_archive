import numpy as np
import matplotlib.pyplot as plt
import pdb, time
import scipy.optimize as opt
from copy import deepcopy
import ifabModel as mdl
import quickhull as qh
import multiprocessing as mp



class WeldPieceInfo:

    LeftEdge = [];
    RightEdge = [];
    Angle = [];
    Points3D = [];


class DataPoint2D_Feature:
    def __init__(self, Points2D):
        self.points2D = Points2D;
        N = np.size(Points2D,0);
        self.MeanPt = np.mean(Points2D,0);
        EP2DCentered = Points2D - np.tile(self.MeanPt,[N,1]);
        [D, V] = np.linalg.eig(np.dot((EP2DCentered.T),(EP2DCentered)));
        
        if (D[1] > D[0]):  # Order the eigenvalues
            V = np.roll(V,1,1);
        
        self.majorAxis = V[:,0];
        rot90 = np.array([[0,-1],[1,0]]);
        
        self.minorAxis = np.dot(rot90,self.majorAxis);
        
        ProjectionMajor = np.dot(EP2DCentered,self.majorAxis);
        ProjectionMinor = np.dot(EP2DCentered,self.minorAxis);
        
        self.Lmin = np.min(ProjectionMajor);
        self.Lmax = np.max(ProjectionMajor);
        self.Wmin = np.min(ProjectionMinor);
        self.Wmax = np.max(ProjectionMinor);
        
        bboxLength_L = self.Lmin*self.majorAxis;
        bboxLength_R = self.Lmax*self.majorAxis;
        
        self.bboxLine = np.vstack((bboxLength_L,bboxLength_R));
        
        bboxUp = self.bboxLine + (self.Wmax*self.minorAxis);
        boxMid = ((self.Wmax*self.minorAxis) + (self.Wmin*self.minorAxis))/2
        self.bboxMid = boxMid          
        bboxDown = self.bboxLine + (self.Wmin*self.minorAxis);
        Box = np.vstack((bboxUp, bboxDown));
        #Box = Box + self.MeanPt ;
        self.bbox = qh.qhull(Box);
        #np.delete(self.bbox,4,0)         
        
        self.dataLength = (abs(self.Lmax - self.Lmin));
        self.dataWidth = (abs(self.Wmax - self.Wmin));
        
        self.majorAxisAngle = np.arctan2(V[1,0],V[0,0]); #Angle = atand(V(2,2)/V(1,2));
        self.minorAxisAngle = np.arctan2(V[1,1],V[0,1]);
        
        self.M = np.tan(self.majorAxisAngle); # parameters of line: y = Mx + c
        self.c = self.MeanPt[1] - self.MeanPt[0]*self.M;

        self.pointMin_Major = (((ProjectionMajor == self.Lmin).nonzero())[0])[0];
        self.pointMax_Major = (((ProjectionMajor == self.Lmax).nonzero())[0])[0];
        self.pointMin_Minor = (((ProjectionMinor == self.Wmin).nonzero())[0])[0];
        self.pointMax_Minor = (((ProjectionMinor == self.Wmax).nonzero())[0])[0];
        
#### Custom function to remove rows/columns from a 2-d array

class lineFeatures:
    def __init__(self, p1, p2):
        p1 = np.ravel(p1);
        p2 = np.ravel(p2);
        self.vec = unitVector(p1,p2);
        
        if (p2[0] - p1[0]) == 0: 
            self.m = 1e9 #; print 'zero div'
            self.m2 = 90
    
        else: 
            self.m = (np.arctan(((p2[1] - p1[1])/(p2[0] - p1[0]))))*180/(np.pi);
            self.m2 = np.rad2deg(np.arctan2((p2[1]-p1[1]),(p2[0] - p1[0])));

            if (self.m2 < 0):
                self.m2 = 180 + (self.m2)
            
        if (np.isnan(self.m)): 
            self.m = 0 ;
           

        if (self.m2 == 0):
            self.m2 = 180;
            
        self.c = p2[1] - np.tan(((self.m)*p2[0])*np.pi/180) ; 
        self.centre = (p1+p2)/2;
        self.m3 = np.rad2deg(np.arctan2((p2[1]-p1[1]),(p2[0] - p1[0])));

def splitPosition(p1,p2,q):
    q = np.ravel(q);
    L1 = lineFeatures(p1,p2);
    v = q[1] - (L1.m)*q[0] - L1.c;
    return (np.sign(v))

def Delete_False_Dim ( Logical_Vector, In_Matrix, Axis = 0 ):

    N = len(Logical_Vector);

    if (np.sum(Logical_Vector) > 0):    
    
        N_R = np.arange(0,N);
        To_Keep = N_R[Logical_Vector];

        if (Axis == 0):
            Out_Matrix = In_Matrix[To_Keep,:];
        
        elif ( Axis == 1 ):
            Out_Matrix = In_Matrix[:,To_Keep];
        
        else:
            print " Function Only Meant for 2-D Matrices "
            return
    else :
        Out_Matrix = In_Matrix ;
        
    return Out_Matrix
        
#### Function to Sequence the input 4DPoints (Last dimension corresponds to Sequence Data)


def iFAB_PCL_Sequence( Points4D, pltFlag = 0 ):

    N = Points4D.shape[0]; #N = size(Points4D,1);
    Seq = 0; #Seq = 1;

    SequenceIx = np.arange(0,N);
    SortedSequence = np.zeros(N);
    
    
    # Sequentialize Data from Index Marker

    SeqData = Points4D[:,3];
    SeqData1 = np.roll(SeqData,-1);
    SeqDiff = (SeqData - SeqData1); # Find Difference between adjacent values
    SeqDiff = np.delete(SeqDiff,[SeqDiff.size - 1]);

    # Identify huge jumps as folds

    FoldPos = (SeqDiff > 1024);
    Folds = sum(FoldPos);
    CarryOverFlag = 0;
    IxBuffer = np.array([0]);
    CarryOverBuffer = np.array([0,0,0,0]);
    if (Folds > 0):

        print "\t\t Total Folds Identified : ", Folds, "\n";
        FoldIx = SequenceIx[FoldPos];
        Fd = np.roll(FoldIx,-1);
        
        Overlapped = (abs(FoldIx - Fd) < 100);
        Overlapped[-1] = 0;

        for i in xrange(0,Folds):
            
            if np.logical_not((Overlapped[i])):

                BufferData = Points4D[Seq:(FoldIx[i] + 1),3];   # Get first set
                I = np.argsort(BufferData);                     # Sort picked fold
                SortedSequence[Seq:FoldIx[i] + 1] = I + Seq;    # Save sorted fold
                Seq = FoldIx[i] + 1;                            # Update fold index
                
                if CarryOverFlag: # if previous folds were redundant, act accordingly
                    In_Ix = np.arange(0,IxBuffer.size - 1);
                    Points4D = np.insert(Points4D,In_Ix + Seq,CarryOverBuffer[1:,:],0); # re-insert removd values into next fold
                    CarryOverBuffer = np.array([[0],[0],[0],[0]]); # empty accumulated buffer
                    FoldIx = FoldIx + IxBuffer.size - 1; # update index values
                    IxBuffer = np.array([0]); # reset redundant fold count
                
                if (i==(Folds-1)):                                  # If last fold index, sort the rest

                    I = np.argsort(Points4D[Seq:,3]);
                    SortedSequence[Seq:] = Seq + I;
                
                CarryOverFlag = 0;
                CarryOverBuffer = np.array([0,0,0,0]);
            
            else:
                
                CarryOverFlag = 1; # Mark for adding values into next fold
                IxBuffer = np.hstack((IxBuffer,FoldIx[i]+1));   # Monitor fold overlaps 
  
                CarryOverBuffer = np.vstack((CarryOverBuffer,Points4D[FoldIx[i]+1,:]))  # Accumulate Values at overlapping folds               
                
                B = np.zeros([np.size(Points4D,0),1]); # remove the values at overlapping folds
                B = B+1;
                B[FoldIx[i]+1] = 0;
                B = B.flatten();
                Points4D = np.compress(B,Points4D,0);
                
                FoldIx = FoldIx - 1; # update fold index after popping a value

    else:

        print "\t\t No Folds Identified"
        I = np.argsort(Points4D[:,3]);
        SortedSequence = I;
       

    OutOfSequence = sum(SortedSequence != SequenceIx);
    print "\t\t Points out of Sequence: ", OutOfSequence, "\n";   
    Points3D = Points4D[np.int64(SortedSequence),0:3];
    
    
    

    return Points3D

#### Function to remove noise/ambiguities from the sensor while identifying the End Points


def End_Point_CleanUp(Points3D, EP, EP_Idx):

    EP = deepcopy(EP);
    PData = DataPoint2D_Feature(EP[:,0:2]);
    EP[:,0:2] = pclRotate(EP[:,0:2],PData.majorAxisAngle);
    
    MeanZ = np.mean(Points3D[:,2]);
    MeanY = (max(Points3D[:,1]) - min(Points3D[:,1]))/2;

    EP_Z = EP[:,2];
    Ix = (EP_Z > MeanZ);
    
    EP = Delete_False_Dim(np.logical_not (Ix), EP);
    EP_Idx = Delete_False_Dim(np.logical_not(Ix),EP_Idx);

    if (sum(Ix) > 0 ):
        print "\t\tCondition 1 violated "

    EP_Y = EP[:,1];
    E2 = np.roll(EP_Y,-1);
    
    RangeVal = abs(EP_Y - E2);

    Ix_R = RangeVal < MeanY ;
    Ix_R[-1] = 0;
    Ix_R = np.roll(Ix_R,1);

    EP = Delete_False_Dim(np.logical_not(Ix_R), EP);
    EP_Idx = Delete_False_Dim(np.logical_not(Ix_R), EP_Idx);

    if (sum(Ix_R) > 0 ):
        print "\t\tCondition 2 violated "

    EP[:,0:2] = pclRotate(EP[:,0:2],-PData.majorAxisAngle);
    return (EP, EP_Idx)


#### Function to identify the End Points (EP) for each 'Pass'


def iFAB_PCL_PassEndPoints( Points3D ):

    X_Val = Points3D[:,1];
    N = len(X_Val);
    EP_Idx = np.arange(0,N);
    X_Val2 = np.roll(X_Val,-1);

    Diff1 = X_Val2 - X_Val;
    Diff1 = np.delete(Diff1, [N-1]);

    ValSlope = np.sign(Diff1);
    ####
    ValSlope[ValSlope == 0] = 1;

    Diff2 = (np.roll(ValSlope,-1) - ValSlope);
    Diff2 = np.delete(Diff2,[len(Diff2) - 1]);

    DiffAbs = abs(Diff2);
    V = DiffAbs == 2;
    EP_Idx = EP_Idx[V];
    EP = Points3D[V,:];

    EP = np.vstack((EP, Points3D[-1,:]));
    EP_Idx = np.hstack((EP_Idx,N));

    (EP, EP_Idx) = End_Point_CleanUp(Points3D, EP, EP_Idx);

    return (EP, EP_Idx)

#### Function to extract the Edges of the Work Piece by identifying them as jumps along the Z - Axis


def iFAB_PCL_ExtractEdges ( Points3D, EP_Idx ):

    Edge1 = EP_Idx;
    Edge2 = np.roll(EP_Idx,-1);
    MeanZ = np.mean(Points3D[:,2]);

    Edge1 = np.delete(Edge1, [(len(Edge1) -1)]);
    Edge2 = np.delete(Edge2, [(len(Edge2) -1)]);

    N = len(Edge1);

    EdgeArray = np.array([0]);
    Flag = 0;

    for i in range(0,N):
    
        BufferPts = Points3D[Edge1[i]:Edge2[i],2];

        if (len(BufferPts) > 5):

            Idx_Val = np.arange(Edge1[i],Edge2[i]) ;
        
            BufferPts2 = np.roll(BufferPts,-1);
        
            Diff1 = BufferPts2 - BufferPts;
            Diff1 = np.delete(Diff1,[(len(Diff1) - 1)]);
            Diff1 = abs(Diff1);
        
            MeanDiff = np.mean(Diff1);
            StdDiff = np.std(Diff1);
            ZScore = (Diff1 - MeanDiff)/StdDiff;
            OutlierCheck = sum(ZScore > 3);
            
            
             
            Idx = ZScore>3;
            C = Idx_Val[Idx];
            C_Z = Points3D[C,2];
        
            Ix = C_Z < MeanZ;
            #C = C';
            C[Ix] = C[Ix] + 1;
        
            if (OutlierCheck == 2):
                if Flag:
                    C = np.roll(C,1);
                EdgeArray = np.hstack((EdgeArray, C));
#                if (Flag and (EdgeArray.size == 3)):
#                    EdgeArray = np.array([0]);
#                    print "\t\t First Pass missed -- Flipping points to maintain pattern \n";
            else:
                print "\t\tOutlier Detected at pass ", i, "\n"
                Flag = not Flag;

    EdgeArray = np.delete(EdgeArray,[0]);
    return EdgeArray



#### Function to calculate orientations from the extracted edges 

def iFAB_PCL_GetOrientation ( EdgePoints3D ):

    N = np.size(EdgePoints3D,0);
    EdgeL1 = np.arange(0,N,4);
    EdgeL2 = EdgeL1 + 3;
    T_Ix = EdgeL2 > (N-1);
    EdgeL2 = Delete_False_Dim(np.logical_not(T_Ix),EdgeL2);
    
    EdgeLIx = np.hstack((EdgeL1, EdgeL2));
    Ix = np.arange(0,N);
    Ix[EdgeLIx] = 0;
    T_Ix = np.logical_not(Ix == 0);
    Ix = Delete_False_Dim(T_Ix,Ix); #Ix(Ix==0) = [];
    EdgeRIx = Ix;

    EdgeL = EdgePoints3D[EdgeLIx,:];
    EdgeR = EdgePoints3D[EdgeRIx,:];

    MeanL = np.mean(EdgeL,0);
    MeanR = np.mean(EdgeR,0);
      
    
    EdgeL_C = EdgeL - np.tile(MeanL,[np.size(EdgeL,0),1]);
    EdgeR_C = EdgeR - np.tile(MeanR,[np.size(EdgeR,0),1]);

    W = np.vstack((EdgeL_C,EdgeR_C));
    
    Angle_L = iFAB_PCL_Local_FindEdgeOrienataion(EdgeL);
    print "\t\tLeft Edge Orientation: ",Angle_L * (180/np.pi), "\n";

    Angle_R = iFAB_PCL_Local_FindEdgeOrienataion(EdgeR);
    print "\t\tRight Edge Orientation: ",Angle_R * (180/np.pi), "\n";

    Angle_Cumulative = iFAB_PCL_Local_FindEdgeOrienataion(W);
    print "\t\tCumulative Edge Orientation: ", Angle_Cumulative * (180/np.pi), "\n";

    WP = WeldPieceInfo();

    WP.LeftEdge = MeanL;
    WP.RightEdge = MeanR;
    WP.Angle = Angle_Cumulative;

    return WP

#### Function assosciated with the math of edge direction calculation

def iFAB_PCL_Local_FindEdgeOrienataion (EdgePoints3D):

    EP2D = EdgePoints3D[:,0:2];
    N = np.size(EP2D,0);
    MeanPt = np.mean(EP2D,0);
    EP2DCentered = EP2D - np.tile(MeanPt,[N,1]);

    [D, V] = np.linalg.eig(np.dot((EP2DCentered.T),(EP2DCentered)));
    Angle = np.arctan2(V[1,0],V[0,0]); #Angle = atand(V(2,2)/V(1,2));

    return Angle
    
    
    
def iFAB_WorkPiece_Data (Points4D):
    
    P3D = iFAB_PCL_Sequence(Points4D);
    [EP, EP_Idx] = iFAB_PCL_PassEndPoints(P3D);
    EdgeArray = iFAB_PCL_ExtractEdges(P3D, EP_Idx);
    EdgePoint3D = P3D[EdgeArray,:];
    W = iFAB_PCL_GetOrientation(EdgePoint3D);
    W.Points3D = P3D
    return W
    
    
    
def iFAB_Weld_Points (P1, P2):
    
    w2 = iFAB_WorkPiece_Data(P1);
    w1 = iFAB_WorkPiece_Data(P2);

    ML1 = w1.LeftEdge;
    ML2 = w2.RightEdge;
    MR1 = w1.RightEdge;
    MR2 = w2.LeftEdge;

    Z = max([ML1[2], ML2[2], MR1[2], MR2[2]]);

    P1 = w1.Points3D;
    P2 = w2.Points3D;

    L1_m = np.tan(w1.Angle);
    L2_m = np.tan(w2.Angle);
    R1_m = L1_m;
    R2_m = L2_m;

    L1_c = w1.LeftEdge[1] - w1.LeftEdge[0]*L1_m;
    R1_c = w1.RightEdge[1] - w1.RightEdge[0]*R1_m;
    L2_c = w2.LeftEdge[1] - w2.LeftEdge[0]*L2_m;
    R2_c = w2.RightEdge[1] - w2.RightEdge[0]*R2_m;

    X_L = Calculate_Intersection(L1_m,L2_m,L1_c,L2_c);
    X_R = Calculate_Intersection(R1_m,R2_m,R1_c,R2_c);
    
    x_L = Calculate_Intersection(L1_m,R2_m,L1_c,R2_c);
    x_R = Calculate_Intersection(R1_m,L2_m,R1_c,L2_c);
    
    D1  = X_L - X_R;
    D2  = x_L - x_R;
    
    d1 = D1.T * D1;
    d2 = D2.T * D2;
    
    if (d1 < d2):
    	
    	X_R = (np.array(np.vstack((X_R,Z)))).flatten();
    	X_L = (np.array(np.vstack((X_L,Z)))).flatten();
    	return [X_L,X_R]
    else:
    	x_R = (np.array(np.vstack((x_R,Z)))).flatten();
    	x_L = (np.array(np.vstack((x_L,Z)))).flatten();
    	return [x_L,x_R]
  

def Calculate_Intersection (m1,m2,c1,c2):

    A = np.array([[-m1, 1],[-m2, 1]]);
    B = np.vstack((c1,c2));
 
    A = np.matrix(A);
    B = np.matrix(B);    
    
    X_Int = np.linalg.solve(A,B);
    return X_Int

def Calculate_ZigZag_Pattern (model_A, passes=10, widthFactor=0.5, lengthFactor=0.75, Z=-100, plot = 0):

    a = DataPoint2D_Feature(model_A);
        
    if (plot) : plt.plot(model_A[:,0], model_A[:,1], '-bo');
    
    Bx = a.bbox;
    if (plot): plt.plot(Bx[:,0],Bx[:,1],'-ro');
    
    print a.pointMin_Major
    Intersection = a.MeanPt + a.majorAxis*(a.dataLength/1.7);    
   
    a_Away = a.MeanPt - Intersection;
    a_Away = a_Away/(np.sqrt((np.dot(a_Away,a_Away))));
    
    a_Start = a_Away*a.dataLength/3 + Intersection;
    a_End = a_Away*a.dataLength*lengthFactor + a_Start;
    a_offset = a.dataWidth*a.minorAxis*widthFactor;
    
    print a.dataLength;
    print a.dataWidth;
    

    Flip = np.array([[1, 1],[-1, -1]]);
    FlipTot = np.tile(Flip,[2*passes,1]);

    a_offset = np.tile(a_offset,[2*passes,1]);
    n = 3 + (passes - 1)*2;
    N = (2*n) - 1;
    
   
    Idx = np.arange(1,N+1);
    Idx = np.mod(Idx,2)
    Idx = np.logical_not(Idx);
    Extrema = Idx.nonzero();
    
    a_x = (np.linspace(a_Start[0],a_End[0],N))[Extrema];
    a_y = (np.linspace(a_Start[1],a_End[1],N))[Extrema];
    a_extrema = (np.vstack((a_x,a_y))).T;
    
    n1 = a_extrema.shape[0];
    #pdb.set_trace();
    FlipTot = FlipTot[0:n1,:];
    a_extrema = a_extrema + a_offset*FlipTot;
    
    Z = np.tile(Z,[n1,1]);
    a_extrema = np.hstack((a_extrema,Z));
    
    if (plot): plt.plot(a_extrema[:,0], a_extrema[:,1], '-gv')    
    return a_extrema



def point2LineDist (a, p1, p2):

    v = p2-p1;
    m_v = np.sqrt(np.dot(v,v));
    
    if (m_v > 0):
        v_unit = v/m_v;
        v_normal = np.array([-v_unit[1],v_unit[0]]);
        
        ab = a - p1 ;
        a_projection = np.dot(ab,v_unit);
        
        if (a_projection < 0):
            a_dst = np.sqrt(abs(np.dot(ab,ab))) * 1.0; ### penalty for points outside convex hull
        elif (abs(a_projection) > m_v ):
            ab = a - p2;
            a_dst = np.sqrt(abs(np.dot(ab,ab))) * 1.0; ### penalty for points outside convex hull
        else:
            a_projection = abs(np.dot(ab,v_normal));
            a_dst = np.sqrt(a_projection);

    else:
        d1 = a - p1;
        a_dst = np.sqrt(np.dot(d1,d1));
        print "overlap !!"
        
    return(a_dst)


# objective function that returns the current error for the input variable 'X' 

def templateMatchError(X, *args):
    
    # applying calculated translation
    #### we absolutely do not want to modify the template globally #####    
    
    # check for constraints on DOF !!!!!!!!!!!!!!!  Important !!!!!!!!!!! locks movement
    try:
        dofConstraint = args[2];
    except:
        dofConstraint = [1,1,1]

    # apply constraint
    
    X = X*dofConstraint
        
    P = deepcopy(args[0]); 
    Np = np.size(P,0);
    template = deepcopy(args[1]);
    
    #pdb.set_trace();
    
    template[:,0] = template[:,0] + X[0];
    template[:,1] = template[:,1] + X[1];

    # applying calculated rotation
    
    t = X[2]; 
    template = pclRotate(template,t)

    p_n = np.size(P,0);
    t_n = np.size(template,0);
    
    v1 = np.arange(0, t_n , 2);
    v2 = np.arange(1, t_n , 2);

    v11 = np.arange(0, t_n , 4);
    v10 = np.arange(1, t_n, 4);
    v20 = np.arange(2, t_n, 4);
    v21 = np.arange(3, t_n , 4);

    t1 = template[v11,:];
    t2 = template[v21,:];
    t10 = template[v10,:];
    t20 = template[v20,:];
    
    tm1 = (t1 + t2)/2;
    tm2 = (t10 + t20)/2;
    
    tt = np.hstack((tm1,tm2));    
    tm = np.reshape(tt,[-1,2])    
       
    errorSum = 0;    
    error_array = np.zeros([t_n/2]);
    memberIdx = np.zeros([p_n,1]);
   
    for i in xrange(0,p_n):
        
        error_array = error_array * 0;
        for j in xrange(0,t_n/2):
            #pdb.set_trace();
            error_array[j] = point2LineDist(P[i,:],template[v1[j],:],template[v2[j],:]);
        
        closestIdx = error_array.ravel().argsort();
        memberIdx[i] = np.floor(closestIdx[0]/2);
        errorSum = errorSum + np.min(error_array);
    
    memberIdx = np.int64(memberIdx)
    centeredVal = 0;    
    
    #for j in xrange(0,p_n):

    #    centeredVal = centeredVal + splitPosition(tm[memberIdx[j],:], tm[(memberIdx[j] + 1),:], P[j, :]);
    
    errorSum =1*( errorSum + (np.abs(centeredVal))*0)/Np;
    print errorSum    
        
    
    return(errorSum)

def extractPeak ( points4D, plot = 1 ):
    
    p3D = iFAB_PCL_Sequence(points4D);
    N = np.size(p3D,0);
    Idx = np.arange(0,N);
    plt.figure();
    plt.plot(p3D[:,0],p3D[:,1],'ro');
    pZ = p3D[:,2];
    Flag = checkConcavity(pZ);

    if np.logical_not(Flag):
        pZ = (-1)*pZ;
    
    if plot:
        plt.figure();
        plt.plot(pZ[:],'ro')
      
    
    foundFlag = 1;
    dataIntractableCounter = 0
    while ((foundFlag) & (dataIntractableCounter < 10)):
        
        dataIntractableCounter += 1;
        
        pZv = np.vstack((pZ,Idx));
        pZv = pZv.T
        Max = np.max(pZ);
        MaxIdx = Idx[pZ == Max];
        MaxIdx = MaxIdx[0];
        N2 = 30;
        
        while np.logical_and(((MaxIdx - N2) < 0),((MaxIdx + N2) > N)):
            N2 -= 1;
            print 'Shrinking Neighborhood to ', N2
        
        lMembers = pZv[MaxIdx-N2:MaxIdx,:];
        rMembers = pZv[MaxIdx:MaxIdx+N2,:];
        
        leftVar = DataPoint2D_Feature(lMembers)
        rightVar = DataPoint2D_Feature(rMembers)
        
        maxVar = min(leftVar.dataWidth,rightVar.dataWidth);
        
        print maxVar
        print np.rad2deg(leftVar.majorAxisAngle)
        print np.rad2deg(rightVar.majorAxisAngle)
        
        if (maxVar < 2):
            foundFlag = 0;
        else:
            if (MaxIdx > 0):
                pZ[MaxIdx] = pZ[MaxIdx - 1];
            else:
                pZ[MaxIdx] = pZ[MaxIdx + 1];
            
    #plt.plot(pZ,'ro')
    
    if (foundFlag == 0):
        if plot:
            plt.plot(MaxIdx,pZ[MaxIdx],'bo');
            plt.show()
        return p3D[MaxIdx,0:2];
    else:
        print 'could not id peak'
        return np.zeros([2]);

def extractPeak2 (points4D):

    p3D = iFAB_PCL_Sequence(points4D);
    z = p3D[:,2];
    n = np.size(z,0);

    plt.figure();
    plt.plot(z,'bo');
    
    z_d1 = np.roll(z,-1) - z;
    z_d2 = np.roll(z_d1,-1) - z_d1;
    
    plt.figure();
    plt.plot(z_d1,'ro');
    plt.figure();
    plt.plot(z_d2,'bo');
    
    Flag = checkConcavity(z);

    if np.logical_not(Flag):
        z = (-1)*z;

        
    return

def checkConcavity (z):

    N = np.size(z,0);
    f = np.ceil(N/3);
    xf = np.arange(0,f);
    
    d1 = z[0:f];
    d2 = z[N-f : N+1];
    
    D1 = np.vstack((d1,xf));
    D2 = np.vstack((d2,xf));
    
    D1 = np.transpose(D1);
    D2 = np.transpose(D2);

    e1 = DataPoint2D_Feature(D1);
    e2 = DataPoint2D_Feature(D2);
    
    a1 =  np.rad2deg(e1.majorAxisAngle)
    a2 =  np.rad2deg(e2.majorAxisAngle)        
    
    if (a1<0): a1 = 180 + a1
    if (a2<0): a2 = 180 + a2

    flag = np.logical_and( (90-a1)>0,(a2 - 90) > 0)
    print 'Concavity test: ', flag, '\n'
        
    return flag

  
###################### WARNING : templateHeight might change between tasks #############33

def extractEdges (points4D, neighborhood = 3, templateHeight = 50, proximity = 5):
    
    p3D = iFAB_PCL_Sequence(points4D);
    z = p3D[:,2];
    n = z.size;
    z_mean = np.mean(z);
    idx = np.arange(0,n);    

#### Slot issue flag

    p21 = p3D[:,[0,1]];
    p22 = np.roll(p21,1,0);
    
    p_dist = ( p21 - p22 );
    p_dist2 = ( p22 - p21 );
    
    p_dist = ( p_dist * p_dist) ;
    p_dist2 = ( p_dist2 * p_dist2);
    
    pd = np.sum(p_dist,1);
    pd2 = np.sum(p_dist2, 1);
    
    pd = np.sqrt(pd);
    pd2 = np.sqrt(pd2);
    
   # slotReject  = (pd > proximity);
   # reject2 = np.roll(slotReject,1,0);
    
   # slotreject = (slotReject + reject2);
    
    platformSlotFlag = np.logical_not((pd > proximity) | (pd2 > proximity)) ;
    
#### 

    neighborhood1 = 10 
    
    l1 = np.zeros([neighborhood1]) + 1;
    r1 = l1 - 2;
   
    f = np.hstack((l1, [0], r1));    
    f = f/neighborhood1;

    filteredData = abs(np.convolve(z,f,'same'));
    filteredData[0:4] = 0;
    filteredData[n-3:n+1] = 0;
    
    edgeCandidates = (filteredData > 0.9*templateHeight) * (filteredData < 1.1*templateHeight) * platformSlotFlag # * (z > z_mean) ;

####  
    
    l1 = np.zeros([neighborhood]) + 1;
    r1 = l1 - 2;
   
    f = np.hstack((l1, [0], r1));    
    f = f/neighborhood;

    filteredData = abs(np.convolve(z,f,'same'));
    filteredData[0:4] = 0;
    filteredData[n-3:n+1] = 0;
    edges = ((filteredData > 0.9*templateHeight))
    edges = edges*edgeCandidates;
    #edges = z > z_mean

    edgeptsAll = p3D[edgeCandidates,:];
    edgepts = p3D[edges,:];
    
   # plt.figure()
    plt.plot(edgeptsAll[:,0], edgeptsAll[:,1],'ro');
   # plt.figure()
    plt.plot(edgepts[:,0],edgepts[:,1],'ro');
    return edgepts
    
def matchTemplate2Data(points2D, template, x_start = np.zeros([1,3]), fName = 'N', dofConstraint = [0,1,1], plot = 1):
    
    
    template = np.float64(deepcopy(template));    
    points2D = np.float64(deepcopy(points2D));

    if (plot): plotTemplate(template, plotPattern = '-kx')    
    
    x_opt = opt.fmin(templateMatchError,x_start,args = (points2D, template, dofConstraint), maxiter = 100)
    
    #pdb.set_trace();
    x_opt = x_opt*dofConstraint
    
    actual_transformation = transformationMatrix(x_opt, template);    
    
    template[:,0] = template[:,0] + x_opt[0];
    template[:,1] = template[:,1] + x_opt[1];

    t = x_opt[2]; 

    template = pclRotate(template,t);
    
    if (plot): plotTemplate(template)
    if (plot): plt.plot(points2D[:,0],points2D[:,1],'ro');
    
    plt.axes().set_aspect('equal','datalim')
    #plt.show()
    x_start[:] = x_opt;
    
    saveData = 1;

    #ct = time.ctime();
    #tStamp = ct[8:10] + ct[11:13] + ct[14:16]

    fDataName = 'C:\\Users\\Ares\\Documents\\ifab\\main\\FitData_' + fName + '.txt'  
    fErrorName = 'C:\\Users\\Ares\\Documents\\ifab\\main\\FitError_' + fName + '.txt'
    fAvgErrorName = 'C:\\Users\\Ares\\Documents\\ifab\\main\\AvgError_' + fName + '.txt' 
    
    ####################################################################################
    
    if (saveData):
        try:
            File1 = open(fDataName,'a');
        except:
            File1 = open(fDataName,'w');
        
        try:
            File2 = open(fErrorName,'a');
        except:
            File2 = open(fErrorName,'w');
    
        try:
            File3 = open(fAvgErrorName,'a');
        except:
            File3 = open(fAvgErrorName,'w');            
    
        np.savetxt(File1,np.mat(x_start), fmt = '%5.5f')
        File1.close();
       
     
    ##################################################################################
    #global memberIdx
    #print memberIdx
    
    return actual_transformation

def matchTemplate2Data_multiprocess(points2D, template, x_start, qTransform, dofConstraint = [1,1,1], plot = 0):
    
    
    template = np.float64(deepcopy(template));    
    points2D = np.float64(deepcopy(points2D));

    if (plot): plotTemplate(template, plotPattern = '-kx')    
    
    ##################################### DOF Constraint WARNING ###########################    
    x_opt = opt.fmin_powell(templateMatchError,x_start,args = (points2D, template, dofConstraint))
    x_opt = x_opt*dofConstraint
    #################################### WARNING ###########################################
    
    template[:,0] = template[:,0] + x_opt[0];
    template[:,1] = template[:,1] + x_opt[1];

    t = x_opt[2]; 

    template = pclRotate(template,t);
    
    if (plot): plotTemplate(template)
    if (plot): plt.plot(points2D[:,0],points2D[:,1],'ro');
    
    if (plot) : plt.axes().set_aspect('equal','datalim')
    if (plot) : plt.show()
    #x_start[:] = x_opt;
    qTransform.put(x_opt);

    return    

def multiCoreTemplateFit (points3D, templateOrig, cores = 4, trans = 50, rot = 15, plot = 1, flip = 0, saveData = 1, dofConstraint = [0,0,0]):
    
    points2D = points3D[:,[0,1]];
    Tot = np.size(templateOrig,0);
    bfIdx = np.arange(0,4);
    Ln = np.arange(4,Tot,8)
    longIdx = 0;
    for lx in Ln:
        bfStack = bfIdx + lx
        longIdx = np.hstack((longIdx,bfStack));
    
    longIdx = np.delete(longIdx,0);

    template = deepcopy(templateOrig);



    if (cores < 4 ):
        print " Minimum active Core(s) is 4 ";
        cores = 4;
    
    t0 = time.time();
    
    if (flip): template = flipTemplate(template);
        
   # if (plot): plotTemplate(template, plotPattern = '-kx')    
    
    tmpFeature = DataPoint2D_Feature(template);
    tmpMean = tmpFeature.MeanPt;
    
############ multiple start points
    
    qTransform = mp.Queue(maxsize = cores);
    
    start_x = np.random.randint(-trans,trans,[cores,1]);
    start_y = np.random.randint(-trans,trans,[cores,1]);
    start_d = np.zeros([cores,1]);
    #start_d = (np.float64(start_d))*(np.pi/180);
    
    x_start = np.hstack((start_x,start_y,start_d));
    
############ one of the initial positions is over the centroid and orientation of the data    
    
    pFeature = DataPoint2D_Feature(points2D);
    pMean = (pFeature.MeanPt - tmpMean)
    
    x_start[0,0] = 0;
    x_start[0,1] = 0;
    x_start[0,2] = 0;
    
############ Run paralell optimizations 

    for i in xrange (0,cores):
        
        print " Activating Optimization in Core : ", i, "\n";
        c0 = mp.Process(target = matchTemplate2Data_multiprocess, args = (points2D, template, x_start[i,:], qTransform, dofConstraint));
        c0.start();
        if (plot): plt.plot(x_start[i,0],x_start[i,1],'go');
        if (plot): plt.plot(tmpMean[0], tmpMean[1], 'kv');
        
############ CPU breather

    bufferSize = qTransform.qsize();
    while (np.logical_not(qTransform.full())): 
        
        
        if (qTransform.qsize() > bufferSize):
            print "\t completed optimization in ", qTransform.qsize(), "core(s) \n" ;
            bufferSize = qTransform.qsize();
        
        time.sleep(0.01)
    
########### Pick Best Fit
    
    
    t1 = time.time();
    print "optimization time on", cores, " cores : ", t1 - t0;
    print "solution errors :"
    
    Npt = np.size(points2D,0);
    
    coreError = np.zeros([cores,1]);
    coreTransform = np.zeros([cores,3]);
    pZ = points3D[:,2];
    
    pointsSurf = (pZ > np.mean(pZ));
    pointsFloor = np.logical_not(pointsSurf);
    
    if (plot) : plt.plot(points2D[pointsSurf,0],points2D[pointsSurf,1],'ro');
    if (plot) : plt.plot(points2D[pointsFloor,0],points2D[pointsFloor,1],'ro');
    
    
    
    for j2 in xrange (0,cores):
        
        coreTransform[j2,:] = qTransform.get(j2);
        coreError[j2] = templateMatchError(coreTransform[j2], points2D, template);
        print j2, "\t", coreError[j2], "\n"
    
    print "Calculating best fit \n"
    sortIdx = coreError.ravel().argsort();
    bestFit = coreTransform[sortIdx[0]];
    avgError = coreError[sortIdx[0]]/Npt ;
    
#########################################################    

    """ if (saveData):
        try:
            File1 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\FitData.txt','a');
        except:
            File1 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\FitData.txt','w');
        
        try:
            File2 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\FitError.txt','a');
        except:
            File2 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\FitError.txt','w');
    
        try:
            File3 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\AvgError.txt','a');
        except:
            File3 = open('C:\\Users\\Ares\\Documents\\ifab\\main\\AvgError.txt','w');            
    
        np.savetxt(File1,np.mat(bestFit), fmt = '%5.5f')
        File1.close();
       
        np.savetxt(File2, coreError[sortIdx[0]], fmt = '%d');
        File2.close();

        np.savetxt(File3, avgError, fmt = '%f');
        File3.close();"""

#########################################################        
    
    
    print "Recorded Edges: ", Npt;
    print "Best Fit Error: ", coreError[sortIdx[0]], "\n\n" 
    print "Best Fit Value: ", bestFit, "\n\n"
    print "Average Error: ", coreError[sortIdx[0]]/Npt
   
    actual_transform = transformationMatrix(bestFit, template);
    
    
    ############ for visualization purposes only ###########

    if(plot):
        
        template[:,0] = template[:,0] + bestFit[0];
        template[:,1] = template[:,1] + bestFit[1];

        t = bestFit[2]; 

        template = pclRotate(template,t);
    
        plotTemplate(template)
           
        plt.axes().set_aspect('equal','datalim')
        
    ################# End of visualization ##################

    
    return actual_transform

def pclRotate(points2D, radians, Centroid = 0):
    """ Rotates the given data points (nx2) by radians counterclockwise. 
    Note that this is performed on the input data directly """
    if (np.size(Centroid) == 1):
        mean_p = np.mean(points2D,0);
    else:
        mean_p = Centroid
        
    n = np.size(points2D,0);
    
    ctrP = points2D - np.tile(mean_p,[n,1]);
    t = np.float(radians);
    rotmat = np.array([[np.cos(t), -np.sin(t)],[np.sin(t), np.cos(t)]]);
    points2D = np.dot(rotmat,ctrP.T)    
    points2D = points2D.T
    points2D = points2D + np.tile(mean_p,[n,1]);
    
    return points2D
    
def templateOutline(templateList, plot = 0, keepAll = 1):
    
    N = np.size(templateList,0);
    templateEdge = np.zeros([1,4]);
    for i in xrange(0,N):
        
        v1 = templateList[i];
        v1 = np.delete(v1, [np.size(v1,0) - 1], 0)
        v2 = np.roll(v1,1,0);
        v3 = v1 - v2;                       # vector between adjacent vertices
        
        dist = np.sum((v3*v3),1);       # distance between adjacent points
        n1 = np.size(dist);        
        
        sortIdx = dist.ravel().argsort();       # sort distances 
       # keepList = sortIdx[[n1 - 2, n1-1]];      # keep the longest two 
        
        if (keepAll): keepList = sortIdx;
        else: keepList = sortIdx[[n1 - 2, n1-1]];
       
        bufferList = np.hstack((v1[keepList,:], v2[keepList,:]));
        templateEdge = np.vstack((templateEdge,bufferList));
        
        if (plot):
            
            for j in xrange(0,np.size(bufferList,0)):
                
                x = np.array([0,2]);
                y = x + 1;
                
                plt.plot(bufferList[j,x],bufferList[j,y],'-ro');
        
    templateEdge = np.delete(templateEdge,0,0);
    templateEdge = np.reshape(templateEdge, [-1,2])
    
    return templateEdge;
    
def vectorMagnitude(v):
    m = np.sqrt(np.sum(v*v));
    return m

def unitVector(p1,p2):

    v_p1 = p2 - p1;
    m = vectorMagnitude(v_p1)
    v_unit = v_p1/m;
    return(v_unit)    

def modelCorners ( vertices, faces ):
    N = np.size(vertices,0);
    V_count = np.zeros(N);
    f = np.size(faces,0);
    
    for i in xrange(0,f):

        idx = faces[i,:];
        V_count[idx] = V_count[idx] + 1;
    
    return(V_count)

def cleanHull (hull2D):
    n = np.size(hull2D,0);
    clHull = hull2D[0,:];
    #print n
    for i in xrange (0,n-2):
        
        p1 = hull2D[i,:];
        p2 = hull2D[i+1, :];
        p3 = hull2D[i+2, :];
        
        v2_unit = unitVector(p1,p2);
        v3_unit = unitVector(p2,p3)
        
        attribute = np.arccos(np.dot(v3_unit,v2_unit))*(180/np.pi);
        #print attribute, "\n"
        if (attribute > 5):
            clHull = np.vstack((clHull,p2));
        
    return clHull

def plotTemplate(template, plotPattern = '-bx'):
    
    n = (np.size(template,0))/2;
    t_n = np.size(template,0);

    v1 = np.arange(0, t_n , 4);
    v10 = np.arange(1, t_n, 4);
    v20 = np.arange(2, t_n, 4);
    v2 = np.arange(3, t_n , 4);

    t1 = template[v1,:];
    t2 = template[v2,:];
    t10 = template[v10,:];
    t20 = template[v20,:];
    tm1 = (t1 + t2)/2;
    tm2 = (t10 + t20)/2;
    tt = np.hstack((tm1,tm2));    
    tm = np.reshape(tt,[-1,2])
    
    for i in xrange(0,n):
        x = np.array([2*i, (2*i) + 1]);
        plt.plot(template[x,0],template[x,1], plotPattern);
    
    for i in np.arange(0,np.size(tm,0),2):
        cc = np.array([i, (i) + 1]);
        plt.plot(tm[cc,0], tm[cc,1],'-g*');
    
    plt.axes().set_aspect('equal', 'datalim')
    
    return

def transformationMatrix(x_opt,template):
    
    ########################################
    
    #x_opt = x_opt*0

    ########################################
    tmp = DataPoint2D_Feature(template);
    Offset = tmp.MeanPt;

    tr1_mat = np.float64(np.eye(4));
    tr1_mat[0,3] = -Offset[0];
    tr1_mat[1,3] = -Offset[1];
    
    tr2_mat = np.float64(np.eye(4));
    tr2_mat[0,3] = Offset[0];
    tr2_mat[1,3] = Offset[1];
    
    tr_mat = np.float64(np.eye(4));
    tr_mat[0,3] = x_opt[0];
    tr_mat[1,3] = x_opt[1];
    
    rot_mat = np.float64(np.eye(4));
    rot_mat[0,0] = np.cos(x_opt[2]);
    rot_mat[0,1] = -np.sin(x_opt[2]);
    rot_mat[1,0] = np.sin(x_opt[2]);
    rot_mat[1,1] = np.cos(x_opt[2]);
    
    transformation_matrix = np.dot(rot_mat,tr1_mat);
    transformation_matrix = np.dot(tr2_mat, transformation_matrix);
    transformation_matrix = np.dot(tr_mat, transformation_matrix);
    return (transformation_matrix)

    
def calculateZigZagCompounded (templateFeature, passes=10, widthFactor = 0.75, lengthFactor= 1, Z=-100, plot = 0):

    a = templateFeature;    
    a_Line = a.bboxLine + a.MeanPt + a.bboxMid

    a_Start = a_Line[0,:];
    a_End = a_Line[1,:];
    
    a_offset = a.dataWidth*a.minorAxis*widthFactor;

    if  ( plot ): 
        plt.plot(a.MeanPt[0], a.MeanPt[1], 'bo')
        plt.plot(a_Start[0], a_Start[1],'ro');
        plt.plot(a_End[0], a_End[1],'ro');        
    
    Flip = np.array([[1, 1],[-1, -1]]);
    FlipTot = np.tile(Flip,[2*passes,1]);
    a_offset = np.tile(a_offset,[2*passes,1]);
    n = 3 + (passes - 1)*2;
    N = (2*n) - 1;
       
    Idx = np.arange(1,N+1);
    Idx = np.mod(Idx,2)
    Idx = np.logical_not(Idx);
    Extrema = Idx.nonzero();
    
    a_x = (np.linspace(a_Start[0],a_End[0],N))[Extrema];
    a_y = (np.linspace(a_Start[1],a_End[1],N))[Extrema];
    a_extrema = (np.vstack((a_x,a_y))).T;
    
    n1 = a_extrema.shape[0];
    FlipTot = FlipTot[0:n1,:];
    a_extrema = a_extrema + a_offset*FlipTot;
    Z = np.tile(Z,[n1,1]);
    a_extrema = np.hstack((a_extrema,Z));
    if  (plot): plt.plot(a_extrema[:,0], a_extrema[:,1], '-go')

    return (a_extrema)


def ft(t, a=1.0, vec=[.1,0], p0=[0,0]):

    p = [(a * np.cos(t) + vec[0]*t), (a * np.sin(t) + vec[1]*t)]
    if vec[0] == 0: at = np.pi
    else: at = np.arctan((vec[1]/vec[0])) + np.pi
    pset = -1*np.array([(a * np.cos(at) + vec[0]*at), (a * np.sin(at) + vec[1]*at)])

    return p + pset + p0


def dist(a,b):
    return np.sqrt(np.sum((np.array(a)-b)**2))


def spiral(p0=[1500,0], vec=[1,0], d=600.0, a=50, cnt=15, tseg=(np.pi/6), z=-100):

    t0 = 0; t = t0; tf = np.pi*cnt*2+tseg; path = [];
    vec = (vec/np.linalg.norm(vec)) * ((d-2*a)/tf)
    while t <= tf:
        v = np.append(ft(t, a, vec, p0), z)
        path.append(v)
        t += tseg
    path = np.array(path);     

    return path

def compundedTaskInspection(template, plot = 1, scount = 16, wf = 100):
    
    a1 = DataPoint2D_Feature (template);
    plotTemplate(template)
    b = a1.bbox;
    c = a1.bboxLine;
    c = c + a1.MeanPt + a1.bboxMid   
    #plotTemplate(b + a1.MeanPt);
    plt.plot(c[:,0], c[:,1], '-gv')
    insp = spiral(c[0], (c[1]-c[0]), a1.dataLength, (wf+(a1.dataWidth/2)), scount)
    plt.plot(insp[:,0], insp[:,1], '-g')

    return insp 
    
def flipTemplate(template, axis = 0):

    tmpFeature = DataPoint2D_Feature(template);
    tmpMean = tmpFeature.MeanPt;
    flipTmp = template - tmpMean;
    
    if (axis == 0):
        flipTmp[:,0] = (-1)*flipTmp[:,0];
    elif (axis == 1):
        flipTmp[:,1] = (-1)*flipTmp[:,1];
    else :
        print "Invalid dimension to flip over \n"
    
    flipTmp = flipTmp + tmpMean;
    
    return flipTmp


def deleteDiagonal(X):
    [r,c] = np.shape(X);
    bufferArray = np.zeros([r-1,c]);
    
    for i in xrange(0,c):
        b = X[:,i];
        bufferArray[:,i] = np.delete(b,i);
    

    return(bufferArray)



def pdist2(X, Xc, norm = 'L2'):
    
    try:
        [r,c] = np.shape(X);
        [r2, c2] = np.shape(Xc);
    except:
        print "input should be a 2D array"
        return
    
    if (c <> c2): 
        print "Vector dimensions don't match \n\n"
        return
    
    X0 = np.tile(X,[1,r2]);
    
    st = 1;   
    
    for j in xrange(0,r2):
        if (st):
            
            X1 = np.tile(Xc[j,:],[r,1]);
            st = 0;
        else:
            
            bfr = np.tile(Xc[j,:],[r,1]);
            X1 = np.hstack((X1,bfr));
    
   
    if (norm == 'L1'):
        Xdist = abs(X0 - X1);

    elif (norm == 'L2' ):        
        Xdist = (X0 - X1);
        Xdist = Xdist*Xdist;

    if (c == 1):
        if (norm == 'L2'):
            Xdist = np.sqrt(Xdist);

    elif (c > 1 ):
    
        Xdist = np.reshape(Xdist,[-1,c])
        Xdist = np.sum(Xdist,1);
        
        if (norm == 'L2'): Xdist = np.sqrt(Xdist);
            
        Xdist = np.reshape(Xdist,[-1,r2]);
    
    return(Xdist)

def extractWelds(template, plot = False):
    #print 'input template', template
    
    template = np.array(template) # 8m x 2 dimension - members clockwise
    n = np.size(template,0);
    edgeSlopes = np.zeros([n/2,1]) # only 4m slopes since one member = 2pts
    edgeCentre = np.zeros([n/2,2]);
    edgePair = np.arange(0,n,2); # Couple adjacent Pts as a single line
    candidateIdx = edgePair;
    
    buttJointCounter = 0;
    
    "Get orientation info from all the edges"
    
    st = 1    
    c = 0
    
    for i in edgePair :
        bufferLine = lineFeatures(template[i,:],template[i+1, :]); # extract line information from the edge pairs
        edgeSlopes[c,0] = abs(bufferLine.m); # populate slope data for each member 
        edgeCentre[c,:] = bufferLine.centre; 
        c += 1
        
    "compare slopes and identify parallels"   
        
    
    slopeDiff = pdist2(edgeSlopes, edgeSlopes, norm = "L1"); # compare the extracted slope values
    [r,c] = np.shape(slopeDiff); 
    ignr = (np.eye(r,c))*999;
    edgeParallels = (slopeDiff + ignr) < 5; # this is a 4m x 4m matrix representing slope similarity
    
    weldLines = []
    sideAngles = []
    buttJointAngle = [];
    
    for j in edgePair:
        
        # get one of the points on the edge

        a1 = template[j,:];
        a2 = template[j+1,:];
        
        # find lines parallel to this edge

        weldCandidates = candidateIdx[edgeParallels[j/2,:]] # look for similar slopes for a particular member
        cdt = np.size(weldCandidates);        
        
        if (cdt > 0):
            for j2 in weldCandidates:

                # points on the line segment
                
                p1 = template[j2,:];
                p2 = template[j2+1,:];

                # distance between line segment and point

                rcp1 = point2LineDist(p1,a1,a2);
                rcp2 = point2LineDist(p2,a1,a2);
                
                                
                overlapDist = point2LineDist(a1,p1,p2) + point2LineDist(a2,p1,p2); # (a1,a2) and (p1,p2) are the line segments of interest
                if (overlapDist < 2):
                       
                    # avoid going over the same weld again            
                    edgeParallels[j2/2,j/2] = False;
    
                    # extract and populate weld lines 
                    
                    if (st): 
                        bfr = np.hstack((template[j,:], template[j+1,:]));
                        weldLines = bfr;
                        st = 0
                    else:
                        bfr = np.hstack((template[j,:], template[j+1,:]));
                        weldLines = np.vstack((weldLines,bfr));
                    # check for butt-joint here #
                    
                    ch1 = min(dist(a1,p1), dist(a1,p2)); 
                    ch2 = min(dist(a2,p1), dist(a2,p2));
                    bJointCheck = ch1 + ch2;
                    
                    if (bJointCheck > 10):
                        # case of butt-joint
                        buttJointCounter += 1 
                        overlapEdge = min(ch1,ch2);
                        Flag180 = abs(a1[0] - a2[0]) < 1;
                        print "flag 180 ", Flag180
                        if ((overlapEdge < 10)|(Flag180)):
                            bjAngle = 999;
                        else:
                            bjSlope1 = edgeSlopes[j/2];
                            sequenceCheck = np.mod((j/2),4);
                            
                            ####3
                            bjVec1 = unitVector(a1,a2);
                            if (a1[1] > a2[1]):
                                bjVec1 = (-1)*bjVec1;
                        
                            if (sequenceCheck == 3):
                                bjSlope2 = edgeSlopes[((j-4) / 2)];
                                slopeCentre = edgeCentre[(j-4)/2,1];
                                bjVec2 = unitVector(template[j-4,:],template[j-3,:]);
                                if (template[j-4,1] > template[j-3,1]): bjVec2 = (-1)*bjVec2
                                
                            elif(sequenceCheck == 0):
                                bjSlope2 = edgeSlopes[((j+4) / 2)];
                                slopeCentre = edgeCentre[(j+4)/2,1];
                                bjVec2 = unitVector(template[j+4,:],template[j+5,:]);
                                if (template[j+4,1] > template[j+5,1]):
                                    bjVec2 = (-1)*bjVec2
                                
                                #print bjSlope2, "\t counter reset ", "\t", j+2
                            elif(sequenceCheck == 1):
                                bjSlope2 = edgeSlopes[((j+2)/2)];
                                slopeCentre = edgeCentre[(j+2)/2,1];
                                bjVec2 = unitVector(template[j+2,:],template[j+3,:]);
                                if (template[j+2,1] > template[j+3,1]): bjVec2 = (-1)*bjVec2
                                
                            elif(sequenceCheck == 2):
                                bjSlope2 = edgeSlopes[((j-2)/2)];
                                slopeCentre = edgeCentre[(j-2)/2];
                                bjVec2 = unitVector(template[j-2,:],template[j-1,:]);
                                if (template[j-2,1] > template[j-1,1]): 
                                    bjVec2 = (-1)*bjVec2
                                
                            if (bjSlope1 > 90):
                                bjSlope1 = bjSlope1 - 180;
                            if (bjSlope2 > 90):
                                bjSlope2 = bjSlope2 - 180;

                            centreCheck = max(bfr[1],bfr[3]) < slopeCentre ; 
                            
                            if np.all(centreCheck):
                               
                                bjAngle = (abs(bjSlope2 - bjSlope1))/2  + min(bjSlope1,bjSlope2) ;
                                if (bjVec2[0] < 0): bjVec2 = (-1)*bjVec2;
                            else:
                                bjAngle = 90 + (abs(bjSlope2 - bjSlope1))/2  + min(bjSlope1,bjSlope2) ;  
                                if (bjVec2[0] > 0): bjVec2 = (-1)*bjVec2;
                                    
                            diffVector = bjVec1 - bjVec2;
                            bjAngle = np.rad2deg(np.arctan2(diffVector[1],diffVector[0]))
                    else:
                        
                        bjAngle = 0;
                    
                    if (np.size(buttJointAngle) == 0):
                 
                         buttJointAngle = bjAngle;
                    else:
                         buttJointAngle = np.vstack((buttJointAngle, bjAngle));
    
    wldNo = np.size(weldLines)/4
    
    if (wldNo == 0): 
        print "No welds identified \n" ;
        return
        
    else:

        print "\n", buttJointCounter, "Butt-Joints amongst ", wldNo, " Welds"
        wl = np.reshape(weldLines,[-1,2]);
        for i2 in np.arange(0,2*wldNo,2):
                    d = np.array([i2,i2+1]);
                    if (plot): plt.plot(wl[d,0], wl[d,1], '-ko')
        return(wl,buttJointAngle)
    

def analyzeFit(fileName):
    
    Data = np.loadtxt(fileName);
    
    print np.std(Data[:,1])

    dX = Data[:,0] - np.mean(Data[:,0]);
    dY = Data[:,1] - np.mean(Data[:,1]);
    dA = Data[:,2] - np.mean(Data[:,2]);
    
    stdX = np.std(dX); hx = np.arange(-3*stdX,4*stdX,0.5*stdX)
    stdY = np.std(dY); hy = np.arange(-3*stdY,4*stdY,0.5*stdY)
    stdA = np.std(dA); ha = np.arange(-3*stdA,4*stdA,0.5*stdA)
    
    if (stdX == 0): hx = 6
    if (stdY == 0): hy = 6
    if (stdA == 0): ha = 6    

    print "STD on X: ", stdX, '\n';
    print "STD on Y: ", stdY, '\n';
    print "STD on theta: ", stdA, '\n';    
    
    plt.figure()
    plt.hist(dX, label = 'X distribution', bins = hx, normed = False);
    plt.xlabel('Millimeters')
    plt.ylabel('Frequency')
    plt.legend();    
    
    plt.figure()
    plt.hist(dY, label = 'Y distribution', bins = hy, normed = False);
    plt.xlabel('Millimeters')
    plt.ylabel('Frequency')
    plt.legend()    

    plt.figure();
    plt.hist(dA, label = 'Theta distribution', bins = ha, normed = False);
    plt.xlabel('Radians')
    plt.ylabel('Frequency')
    plt.legend()
   
    Data[:,0] = Data[:,0] * 0;
    
    np.savetxt('RepeatbilityTest.txt',Data, fmt = '%5.5f')
    
    return
  
def confirmWeldPath(templateOrig, transformationMat = np.eye(4), normalFactor = 0, lengthFactor = 30):
    
    template = pclRotate(templateOrig,np.pi/2)    
    Centroid = np.mean(template,0);
    weldTuple = extractWelds(template);
    
    welds = weldTuple[0];
    bjCheck = weldTuple[1];
    
    N = np.size(welds,0);
    checkPass = [];
    
    weldCheckIdx = (np.arange(0,N/2))*0;    
    
    cU = [];
    cL = [];
    
    fVal = normalFactor ;
    approachVector = [];
    
    for i in np.arange(0,N,2):
        if (bjCheck[i/2] <> 999):
            
            weldCheckIdx[i/2] = 1;     #### Mark Weld as can be inspected ####                   
            p1 = welds[i,:];
            p2 = welds[i+1,:];
            D = vectorMagnitude(p2-p1);
            D2 = lengthFactor - D;
            print D;
            print D2;
        
            [p1,p2] = extendLineSegment(p1,p2,D2)        
            l1 = lineFeatures(p1,p2);
        
            rotA = l1.m ; 
            rotA = rotA*np.pi/180 ; 
        
            # Always pick the upper half quadrant
            if (p2[0] > p1[0]):
                p3 = p2;
                p2 = p1;
                p1 = p3;
                #lUnit = (-1)*lUnit;
            ##   
            if (p2[0] == p1[0]):
                if (p2[1] < p1[1]):
                     p3 = p2;
                     p2 = p1;
                     p1 = p3;
                    #lUnit = (-1)*lUnit
            
            lUnit = unitVector(p1,p2)
            C = np.vstack((p1,p2));
            
            # check butt-joint id  
            
            if (bjCheck[i/2] == 0):
                C = pclRotate(C,90*np.pi/180);
            
            else: # this is in the case of a butt-joint
                C = pclRotate(C,(-1)*(rotA));
                C = pclRotate(C,(1)*(bjCheck[i/2]*np.pi/180))
                #print bjCheck[i/2], "\n\n"
            
            C = C + lUnit*(D/2)
            appVector = pclRotate(C,90*np.pi/180);
            appVector = unitVector(appVector[0,:],appVector[1,:])
            print appVector
           
            appAng = np.rad2deg(np.arctan2(appVector[1],appVector[0]))
           
            if (appVector[0] < 0):
                appVector = (-1)*appVector;
                
            if (np.size(approachVector,0) == 0):
                approachVector = appVector
            else:
                approachVector = np.vstack((approachVector, appVector))
            
            print "calculated approach ",appAng
            offsetUpper = (fVal)*appVector
            offsetLower = (-1)*(fVal)*lUnit ;
        
            cUpper = C + offsetUpper ;
            cLower = C - offsetUpper ;
        
            if (np.size(cU) == 0):
                cU = cUpper;
            else:
                cU = np.vstack((cU,cUpper));
            
            if (np.size(cL) == 0):
                cL = cLower;
            else:
                cL = np.vstack((cL,cLower));
        else:
            weldCheckIdx[i/2] = 0; #### Mark welds as cannot be inspected ####
            
    N = np.size(cU,0);
    z = np.ones([N, 1]);
    z2 = np.ones([N/2,1]);
    
    cU = pclRotate(cU,-np.pi/2,Centroid);

    approachV2 = approachVector*0;
    approachV2[:,1] = (-1)*approachVector[:,0];
    approachV2[:,0] = approachVector[:,1]
    approachVector = approachV2
    
    template = pclRotate(template,-np.pi/2);
    
    approachVector = np.hstack((approachVector,z2,z2*0));
    
    approachVector = np.dot(transformationMat, approachVector.T);
    approachVector = approachVector.T;
    approachVector = approachVector[:,0:3];    

    cU = np.hstack((cU,z,z));  
    cU = np.dot(transformationMat,cU.T);
    cU = cU.T;
    cU = cU[:,0:3]; 
    plt.figure()
    plotTemplate(templateOrig)
    
    for p in np.arange(0,N,2):
        plt.plot(cU[[p,p+1],0],cU[[p,p+1],1],'-r', lw = 2);
        
    checkList = np.zeros([N/2,3,3])

    for j in xrange(0,N/2):
        
        checkList[j,0,:] = cU[2*j,:];
        checkList[j,1,:] = cU[2*j + 1,:];
        checkList[j,2,:] = approachVector[j,:];
        
    return (checkList, weldCheckIdx)
        
def extendLineSegment(p1,p2,extension = 0):
    
    lUnit = unitVector(p1,p2);
    P1 = p1 + (-1)*(extension/2)*lUnit;
    P2 = p2 + (1)*(extension/2)*lUnit;

    return (P1,P2)    
    
def transformData(bestFit,template,Data):
    
    Data[:,0] = Data[:,0] + bestFit[0];
    Data[:,1] = Data[:,1] + bestFit[1];

    DataTransformed = pclRotate(Data,bestFit[2],template);
    return(DataTransformed)
    
def coupleData(p4D, jntAngle = 45, step = 10, group = 30):
    
    p3D = iFAB_PCL_Sequence(p4D);
    N2 = np.size(p3D,0);
    rOrigin = p3D[0,0:2];
    absoluteXY = p3D[:,0:2];
    relativeXY = absoluteXY - np.tile(rOrigin,[N2,1]);
    
    #zRange = np.max(p3D[:,2]) - np.min(p3D[:,2]);
    
    Dx = np.sqrt(np.sum(relativeXY*relativeXY,1));
    absRange =  np.max(Dx[:]);
    Dx = np.arange(0,N2)
    Dx = Dx*absRange/N2;
    
    D = p3D[:,2];
    D = np.vstack((Dx,D));
    D = np.transpose(D);
    #pdb.set_trace()
    gpCandidate = np.arange(0,N2-group,step);
    count = 0;
    
    candidateIdx = np.zeros(shape = [1,2]);
    candidateAngle = np.zeros(1);
    
    for i in gpCandidate:

        bufferData = D[i:i+group,:];
        bufferInfo = DataPoint2D_Feature(bufferData);
        
        dataSpread = bufferInfo.dataWidth;

        bufferAngle = np.rad2deg(bufferInfo.majorAxisAngle)
        #bufferAngle = np.ceil(bufferAngle);
        
        if (bufferAngle <= 0):
            bufferAngle = bufferAngle + 180
            
        if dataSpread < 2:
            candidateIdx = np.vstack((candidateIdx,bufferInfo.MeanPt));
            candidateAngle = np.vstack((candidateAngle, bufferAngle));
            
        count += 1;
   
    candidateIdx = np.delete(candidateIdx,0,0);
    candidateAngle = np.delete(candidateAngle,0,0);
    
    N3 = np.size(candidateAngle, 0);
    cdIdx = np.arange(0,N3);
    
    angleRedundant = candidateAngle - np.roll(candidateAngle,-1)
    angleRedundant[-1] = 180;
    
    angleRedundant = np.ravel(abs(angleRedundant) > 5);
   
    cdIdx = cdIdx[angleRedundant];
    
    candidateIdx = candidateIdx[cdIdx,:]
    candidateAngle = candidateAngle[cdIdx];
    
    angleCompare = pdist2(candidateAngle, candidateAngle, norm = 'L1')
    
    plt.plot(D[:,0], D[:,1],'ro');
    plt.plot(candidateIdx[:,0],candidateIdx[:,1],'bo')
    
    ps = extractSeam(angleCompare, jntAngle, candidateIdx, candidateAngle)
    plt.plot(ps[:,0], ps[:,1], 'go')
    
    seamPos = ps[0,:];
    #pdb.set_trace()
    seamProximity = np.sum(np.sqrt(np.multiply((D - seamPos),(D - seamPos))),1);
    seamNearest = ((seamProximity == np.min(seamProximity)).nonzero())[0];
    tangiblePt = p3D[seamNearest,0:2]
    
    return (candidateIdx, candidateAngle, tangiblePt);        
    
        
def extractSeam (confMat, jointAngle, candidateIdx, candidateAngle ):

    angleMatch = abs(confMat - jointAngle);
    seamCandidates = (angleMatch) < 5
    
    if (np.sum(seamCandidates[:]) == 0):
        minVal = np.min(angleMatch[:]);
        print 'NOTE : Closest angle config is off the apriori angle by: ', minVal 
        seamCandidates = (angleMatch) < minVal + 1
        
    (rx, cx) = seamCandidates.nonzero();
    N = np.size(rx);
    possibleSeams = np.zeros([1,2]);
    
    for i in np.arange(0,N):
        flag = 1
        
        # solving line intersection with y = mx + c
        
        m1 = np.tan(np.deg2rad(candidateAngle[rx[i]]));
        m2 = np.tan(np.deg2rad(candidateAngle[cx[i]]));
        
        c1 = (-m1)*(candidateIdx[rx[i],0]) + candidateIdx[rx[i],1] ;
        c2 = (-m2)*(candidateIdx[cx[i],0]) + candidateIdx[cx[i],1] ;
        
        try:
            p_Int = np.transpose(Calculate_Intersection(m1,m2,c1,c2));                
        except:
            flag = 0
        
        if flag:
            possibleSeams = np.vstack((possibleSeams, p_Int));
    
    possibleSeams = np.delete(possibleSeams,0,0);
    return (possibleSeams)


        
