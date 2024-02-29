import tty
import sys
import termios
import select
import time

print(sys.version_info[0])

def getKey():
    counter = 1
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    x = ' '
    while True:
            counter += 1
            if x == chr(27):
                break
            else:
                if select.select([sys.stdin,],[],[],1.0)[0]:
                    x=sys.stdin.read(1)[0]
                    print("Got:", x)
                else:
                    x = ' '
                    print("Got:", x)
            

getKey()