# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 00:02:36 2012

@author: Mabaran
"""

import numpy as np
import matplotlib.pyplot as plt
import pdb, time
import scipy.optimize as opt
from copy import deepcopy
import ifabModel as mdl


# objective function that returns the current error for the input variable 'X' 

def templateMatchError(X, *args):
    
    # applying calculated translation
    #### we absolutely do not want to modify the template globally #####    
    P = deepcopy(args[0]);    
    template = deepcopy(args[1]);
    
    #pdb.set_trace();
    template[:,0] = template[:,0] + X[0];
    template[:,1] = template[:,1] + X[1];

    # applying calculated rotation
    t = X[2]; 
    template = pclRotate(template,t)

    p_n = np.size(P,0);
    t_n = np.size(template,0);
    
    v1 = np.arange(0, 2, p_n + 1);
    v2 = np.arange(1, 2, p_n + 1);
    
    errorSum = 0;    
    error_array = np.zeros([t_n]);    
 
    for i in xrange(0,p_n):
        
        error_array = error_array * 0;def templateOutline(templateList, plot = 0):
    
    N = np.size(templateList,0);
    templateEdge = np.zeros([1,4]);
    for i in xrange(0,N):
        
        v1 = templateList[i];
        v1 = np.delete(v1, [np.size(v1,0) - 1], 0)
        v2 = np.roll(v1,1,0);
        v3 = v1 - v2;                       # vector between adjacent vertices
        
        dist = np.sum((v3*v3),1);       # distance between adjacent points
        n1 = np.size(dist);        
        
        print dist , "\n"        
        sortIdx = dist.ravel().argsort();       # sort distances 
        keepList = sortIdx[[n1 - 2, n1-1]];      # keep the longest two 
        
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
        for j in xrange(0,t_n):
            
            error_array[j] = point2LineDist(P[i,:],template[v1[j],:],template[v2[j],:]);
        
        errorSum = errorSum + np.min(error_array);
       
        
    print errorSum, "\n"
    return(errorSum)
    
def extractEdges (points4D, neighborhood = 10, templateHeight = 50):
    
    p3D = iFAB_PCL_Sequence(points4D);
    z = p3D[:,2];
    n = z.size;
    idx = np.arange(0,n);    
    
    l1 = np.zeros([neighborhood]) + 1;
    r1 = l1 - 2;
   
    f = np.hstack((l1, [0], r1));    
    f = f/neighborhood;

    filteredData = abs(np.convolve(z,f,'same'));
    filteredData[0:4] = 0;
    filteredData[n-3:n+1] = 0;
    
    edges = (filteredData > 0.9*templateHeight);
    edgepts = p3D[edges,:];
    
    return edgepts
    
def matchTemplate2Data(points2D, template, x_start = np.array([0,0,0])):
    
    
    template = np.float64(deepcopy(template));    
    points2D = np.float64(deepcopy(points2D));
    x_start = np.float64(x_start)
    #pdb.set_trace();
    x_opt = opt.fmin(templateMatchError,x_start,args = (points2D, template), maxiter = 2000)
    
    plt.plot(template[:,0],template[:,1],'--kx');    
    template[:,0] = template[:,0] + x_opt[0];
    template[:,1] = template[:,1] + x_opt[1];
    t = x_opt[2]; 
    template = pclRotate(template,t);
    
    plt.plot(template[:,0],template[:,1],'-bx');
    plt.plot(points2D[:,0],points2D[:,1],'ro');
    
    return x_opt


def templateOutline(templateList, plot = 0):
    
    N = np.size(templateList,0);
    templateEdge = np.zeros([1,4]);
    for i in xrange(0,N):
        
        v1 = templateList[i];
        v1 = np.delete(v1, [np.size(v1,0) - 1], 0)
        v2 = np.roll(v1,1,0);
        v3 = v1 - v2;                       # vector between adjacent vertices
        
        dist = np.sum((v3*v3),1);       # distance between adjacent points
        n1 = np.size(dist);        
        
        print dist , "\n"        
        sortIdx = dist.ravel().argsort();       # sort distances 
        keepList = sortIdx[[n1 - 2, n1-1]];      # keep the longest two 
        
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