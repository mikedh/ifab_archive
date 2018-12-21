import cv
import numpy as np 
import time
import matplotlib.pyplot as plt

def getCameraInput():
    w1 = cv.NamedWindow("CamTest",1);
    I = cv.CaptureFromCAM(0);
    i = 0;    
    
    while (i < 200):

        b = cv.QueryFrame(I);
        cv.ShowImage("CamTest",b);
        cv.WaitKey(30);
        i =  i + 1;
        print i;

        if (i == 100):
            print "saving Image"
            cv.SaveImage("camCap.png", b);
    
    cv.DestroyAllWindows();
            
    return

def showChessBoardCorners(imageName, size):

    cv.NamedWindow("Corners",1);
    w1 = cv.LoadImage(imageName);
    c1 = cv.FindChessboardCorners(w1,pattern_size = size);
    foundFlag = np.array(c1[0]);
    print foundFlag;
    c2 = np.array(c1[1]);
    cv.ShowImage("Corners", w1);
    cv.WaitKey(250)

    if (foundFlag):
        plt.plot(c2[:,0,], c2[:,1],'rx');
        cv.WaitKey(250);
    
    cv.DestroyAllWindows()  
    
    return    

def showThresholdedImage (src, dst, Threshold, show = True, method = cv.CV_THRESH_BINARY):
    
    n = src.nChannels;
    m = Threshold;
    
    if (n>1):        
        src2 = cv.CreateImage((src.width,src.height),src.depth,1);
        cv.ConvertImage(src,src2,cv.CV_BGR2GRAY);
    else :
        src2 = src;
    
    cv.Threshold(src2, dst, m, 255, method);
    i = 0;
    if (show):
        cv.NamedWindow("Original",1);
        cv.NamedWindow("Thresholded",1);
        while (i<6000):
            cv.ShowImage("Original", src2)
            cv.ShowImage("Thresholded",dst);
            cv.WaitKey(300);
            i = i + 1
    
    return
    
        