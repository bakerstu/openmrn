#!/usr/bin/env python
'''
Read a file from stdin and write it out as individual byte constants

@author: Bob Jacobsen
'''
bytesPerLine = 128

import sys

def main():
    cnt = 0
    comment = ""
    sys.stdout.write("   ")
    while True :
        c = sys.stdin.read(1)
        if c == "" :
            break
        sys.stdout.write(str(ord(c[0:1]))+", ")
        cnt = cnt+1
        if ord(c[0:1]) >= ord(' ') :
            comment = comment+c
        if cnt >= bytesPerLine :
            sys.stdout.write("   // | ")
            sys.stdout.write(comment)
            sys.stdout.write("|\n   ")
            cnt = 0
            comment = ""
    if cnt != 0 :
        sys.stdout.write("   // | ")
        sys.stdout.write(comment)
        sys.stdout.write("|\n   ")    
    sys.stdout.write("0\n")
   
if __name__ == '__main__':
    main()
 