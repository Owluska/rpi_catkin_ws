#! /usr/bin/env python3
import rospy

path = 'src/rpicar/src/scripts/data/'

name = 'text.txt'
f = open(path + name, 'w')
f.write("Hello!")
f.close()

# f = open("test.txt", 'r')
# print(f.read())
# f.close()
# import os

# print('getcwd:      ', os.getcwd())
# print('__file__:    ', __file__)