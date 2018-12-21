# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 23:59:01 2012

@author: Mabaran
"""

import numpy as np
import matplotlib.pyplot as plt
import pdb, time


class DataPoint2D_Feature:
    def __init__(self, Points2D):
        self.points2D = Points2D;
        N = np.size(Points2D,0);
        Ix = np.arange(0,N);
        self.MeanPt = np.mean(Points2D,0);
        EP2DCentered = Points2D - np.tile(self.MeanPt,[N,1]);
        [D, V] = np.linalg.eig(np.dot((EP2DCentered.T),(EP2DCentered)));
        
        if (D[1] > D[0]):  # Order the eigenvalues
            V = np.roll(V,1,1);
        
        self.majorAxis = V[:,0];
        self.minorAxis = V[:,1];
        
        ProjectionMajor = np.dot(Points2D,self.majorAxis);
        ProjectionMinor = np.dot(Points2D,self.minorAxis);
        Lmin = np.min(ProjectionMajor);
        Lmax = np.max(ProjectionMajor);
        Wmin = np.min(ProjectionMinor);
        Wmax = np.max(ProjectionMinor);
        
        self.dataLength = round(abs(Lmax - Lmin));
        self.dataWidth = round(abs(Wmax - Wmin));
        
        self.majorAxisAngle = np.arctan2(V[1,0],V[0,0]); #Angle = atand(V(2,2)/V(1,2));
        self.minorAxisAngle = np.arctan2(V[1,1],V[0,1]);
        
        self.M = np.tan(self.majorAxisAngle); # parameters of line: y = Mx + c
        self.c = self.MeanPt[1] - self.MeanPt[0]*self.M;

        self.pointMin_Major = (((ProjectionMajor == Lmin).nonzero())[0])[0];
        self.pointMax_Major = (((ProjectionMajor == Lmax).nonzero())[0])[0];
        self.pointMin_Minor = (((ProjectionMinor == Wmin).nonzero())[0])[0];
        self.pointMax_Minor = (((ProjectionMinor == Wmax).nonzero())[0])[0];

#### Custom function to remove rows/columns from a 2-d array

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
        F_ix = np.arange(0,FoldIx.size)
        Fd = np.roll(FoldIx,-1);
        
        Overlapped = (abs(FoldIx - Fd) < 100);
        Overlapped[-1] = 0;

        for i in xrange(0,Folds):
            
            if np.logical_not((Overlapped[i])):

                BufferData = Points4D[Seq:(FoldIx[i] + 1),3];   # Get first set
                I = np.argsort(BufferData);                     # Sort picked fold
                SortedSequence[Seq:FoldIx[i] + 1] = I + Seq;    # Save sorted fold
                Seq = FoldIx[i] + 1;                            # Update fold index
                
                if CarryOverFlag: # if previous folds were redundant act accordingly
                    In_Ix = np.arange(0,IxBuffer.size - 1);
                    Points4D = np.insert(Points4D,In_Ix + Seq,CarryOverBuffer[1:,:],0); # re-insert removd values into next fold
                    CarryOverBuffer = np.array([[0],[0],[0],[0]]); # empty accumulated buffer
                    FoldIx = FoldIx + IxBuffer.size - 1; # update index values
                    IxBuffer = np.array([0]); #reset redundant fold count
                
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
    
def Calculate_Intersection (m1,m2,c1,c2):

    A = np.array([[-m1, 1],[-m2, 1]]);
    B = ([[c1],[c2]]);
    
    A = np.matrix(A);
    B = np.matrix(B);    
    
    X_Int = np.linalg.solve(A,B);
    return X_Int
    
def Calculate_ZigZag_Pattern (model_A, passes=5, widthFactor=1.5, lengthFactor=.5, Z=-100):

    a = DataPoint2D_Feature(model_A);
    
    Intersection = a.MeanPt + a.majorAxis*(a.dataLength/2);    
   
    a_Away = a.MeanPt - Intersection;
    a_Away = a_Away/(np.sqrt((np.dot(a_Away,a_Away))));
    
    a_Start = a_Away*a.dataLength/3 + Intersection;
    a_End = a_Away*a.dataLength*lengthFactor + a_Start;
    a_offset = a.dataWidth*a.minorAxis*widthFactor;
    
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
    
    return a_extrema
    
def pclRotate(points2D, radians):
    """ Rotates the given data points (nx2) by radians counterclockwise. 
    Note that this is performed on the input data directly """

    mean_p = np.mean(points2D,0);
    n = np.size(points2D,0);
    
    ctrP = points2D - np.tile(mean_p,[n,1]);
    t = radians;
    rotmat = np.array([[np.cos(t), -np.sin(t)],[np.sin(t), np.cos(t)]]);
    points2D = np.dot(rotmat,ctrP.T)    
    points2D = points2D.T
    points2D = points2D + np.tile(mean_p,[n,1]);
    
    return points2D
    
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