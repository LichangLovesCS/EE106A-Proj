#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Filename      : cube.py
# Author        : Lichang Xu
# Created       : November, 2016
# Last Modified : December, 2016

from sys import exit as Die
try:
    import sys
    from video import webcam
except ImportError as err:
    Die(err)

class Cube:

    def __init__(self):
        pass

    def run(self):
        state = webcam.scan()
        if not state:
            print('\033[0;33m[QBR SCAN ERROR] you did not scan in all 6 sides.')
            print('Please try again.\033[0m')
            Die(1)

def main():
    print('Starting test program...')
    cube = Cube()
    cube.run()

if __name__ == '__main__':main()
