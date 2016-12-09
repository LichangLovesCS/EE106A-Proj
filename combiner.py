#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Filename      : combiner.py
# Author        : Lichang Xu
# Created       : November, 2016
# Last Modified : December, 2016

class Combine:

    def sides(self, sides):
        """Join all the sides together into one single string.

        :param sides: dictionary with all the sides
        :returns: string
        """
        combined = ''
        for face in 'URFDLB':
            combined += ''.join(sides[face])
        return combined

combine = Combine()
