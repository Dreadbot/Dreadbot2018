#!/usr/bin/env python

import argparse

import cv2
import numpy as np
from matplotlib import pyplot as plt

def main():
    ap = argparse.ArgumentParser(description="Compute a histogram for an image")
    ap.add_argument( "-i", "--image", help="Image to compute a histogram for", required=True)

    args= ap.parse_args()

    gethistogram( args.image )

def gethistogram(imagefile):

    img = cv2.imread(imagefile, -1)
    cv2.imshow('GoldenGate',img)

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    color = ('h','s','v')
    cmap = { 'h':'b', 's':'g', 'v':'r' }
    for channel,col in enumerate(color):
        #histr = cv2.calcHist([img],[channel],None,[256],[0,256])
        histr = cv2.calcHist([hsv],[channel], None, [256],[0,256])

        plt.plot(histr,color = cmap[col])
        if col == 'h':
            plt.xlim([0,180])
        else:
            plt.xlim([0,256])            
        plt.title('Histogram for %s' % col)
        plt.show()

    while True:
        k = cv2.waitKey(0) & 0xFF     
        if k == 27: break             # ESC key to exit 
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    
