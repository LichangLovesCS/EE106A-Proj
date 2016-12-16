#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Filename      : colordetection.py
# Author        : Lichang Xu
# Created       : November, 2016
# Last Modified : December, 2016

from sys import exit as Die
try:
    import sys
except ImportError as err:
    Die(err)

class ColorDetection:

    def get_color_name(self, hsv):
        """ Get the name of the color based on the hue.
            Need adjustment for each different color scheme.

        :returns: string
        """
        (h,s,v) = hsv
        print((h,s,v))
        if (h>100 and h<115) and (s>125 and s<210) and (v < 85 and v > 45):
            return 'blue'
        elif (h>15 and h<35) and (s>180 and s<235) and (v < 93 and v > 67):
            return 'yellow'
        elif (h>45 and h<83) and (s>105 and s<220) and (v < 65 and v > 30):
            return 'green'
        if (h>0 and h<20) and (s>190 and s<235) and (v < 125 and v > 98):
            return 'orange'
        elif (h>0 and h < 90 )  and (s>180 and s<240) and (v < 130 and v > 90):
            return 'red'
        # if h <= 10 and v > 100:
            
        # elif (h>160 and h<180) and (s>0 and s<30) and (v < 255 and v > 221):
        # elif h <= 30 and s <= 100:
            # return 'white'



        return 'white'

    def name_to_rgb(self, name):
        """
        Get the main RGB color for a name.

        :param name: the color name that is requested
        :returns: tuple
        """
        color = {
            'red'    : (0,0,255),
            'orange' : (0,165,255),
            'blue'   : (255,0,0),
            'green'  : (0,255,0),
            'white'  : (255,255,255),
            'yellow' : (0,255,255)
        }
        return color[name]

    def average_hsv(self, roi):
        """ Average the HSV colors in a region of interest.

        :param roi: the image array
        :returns: tuple
        """
        h   = 0
        s   = 0
        v   = 0
        num = 0
        for y in range(len(roi)):
            if y % 10 == 0:
                for x in range(len(roi[y])):
                    if x % 10 == 0:
                        chunk = roi[y][x]
                        num += 1
                        h += chunk[0]
                        s += chunk[1]
                        v += chunk[2]
        h /= num
        s /= num
        v /= num
        return (int(h), int(s), int(v))

ColorDetector = ColorDetection()
