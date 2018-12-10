
# A* Shortest Path Algorithm
# http://en.wikipedia.org/wiki/A*
# FB - 201012256
from heapq import heappush, heappop # for priority queue
import math
import time
import random
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
img_counter = 0
i=0
b=0
r=0
z=80
matrix = np.zeros((6,6), dtype=np.int)
while(1):

    # Take each frame
    _, frame = cap.read()

    frame= frame[0:480,0:480] 
    # Convert BGR to HSV
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for i in range(0,480,80):
        cv2.line(frame,(0,i),(480,i),(255,0,0),2)
        cv2.line(frame,(i,0),(i,480),(255,0,0),2)
    # define range of blue color in HSV
   
    # Threshold the HSV image to get only blue colors
   
    cv2.imshow("abc",frame)
    g = cv2.waitKey(1)
    if g%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif g%256 == 32:
        # SPACE pressed
        img_name = "{}.jpg".format(100)
        cv2.imwrite(img_name, frame)
        r=1
    if (r==1):
        r=0
        img = cv2.imread("{}.jpg".format(100))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0,0,200])
        upper_blue = np.array([10,10,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        for i in range(0,480,80):
            a=0
            for j in range(0,480,80):
                row=i/80
                column=j/80
                crop_img = mask[a:j+z, b:i+z]
                nzCount = cv2.countNonZero(crop_img);
                #print(nzCount)
                if nzCount>1:
                    matrix[row,column]=1
                    print("count=","i=","j=",nzCount,row,column)
                #cv2.imshow("cropped", crop_img)
                #img_name = "{}.jpg".format(img_counter)
                #cv2.imwrite(img_name, crop_img)
                #print("{} written!".format(img_name))
                img_counter += 1
                a +=z
            b +=z
       # print(matrix)
        break
    #for j in range(0,480,80):
        #first_frame = mask[0:80, 0:80]
        #nzCount = cv2.countNonZero(first_frame);
        #print(nzCount)

    #k = cv2.waitKey(5) & 0xFF
    #if k == 27:
        

cv2.destroyAllWindows()    

def abcd(v,w):
    if v==0 and w==1:
        #Right
        print('right')
    if v==1 and w==1:
        #diag_right_dwn
        print('right_dwn')
        
    if v==1 and w==-1:
        #diag_left_dwn
        print('lft_dwn')
        
    if v==0 and w==-1:
        #left
        print('left')
        
    if v==-1 and w==-1:
        #diag_left_up
        print('left_up')
        
    if v==-1 and w==1:
        #diag_right-up
        print('right_up')
        
    if v==-1 and w==0:
        #up
        print('up')
        
    if v==1 and w==0:
        #down
        print('down')
    




        
class node:
    xPos = 0 # x position
    yPos = 0 # y position
    distance = 0 # total distance already travelled to reach the node
    priority = 0 # priority = distance + remaining distance estimate
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other): # comparison method for priority queue
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give higher priority to going straight instead of diagonally
    def nextMove(self, dirs, d): # d: direction to move
        if dirs == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10
    # Estimation function for the remaining distance to the goal.
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        # Euclidian Distance
        d = math.sqrt(xd * xd + yd * yd)
        # Manhattan distance
        # d = abs(xd) + abs(yd)
        # Chebyshev distance
        # d = max(abs(xd), abs(yd))
        return(d)

# A-star algorithm.
# The path returned will be a string of digits of directions.
def pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB):
    closed_nodes_map = [] # map of closed (tried-out) nodes
    open_nodes_map = [] # map of open (not-yet-tried) nodes
    dir_map = [] # map of dirs
    row = [0] * n
    for i in range(m): # create 2d arrays
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], []] # priority queues of open (not-yet-tried) nodes
    pqi = 0 # priority queue index
    # create the start node and push into list of open nodes
    n0 = node(xA, yA, 0, 0)
    n0.updatePriority(xB, yB)
    heappush(pq[pqi], n0)
    open_nodes_map[yA][xA] = n0.priority # mark it on the open nodes map

    # A* search
    while len(pq[pqi]) > 0:
        # get the current node w/ the highest priority
        # from the list of open nodes
        n1 = pq[pqi][0] # top node
        n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        heappop(pq[pqi]) # remove the node from the open list
        open_nodes_map[y][x] = 0
        closed_nodes_map[y][x] = 1 # mark it on the closed nodes map

        # quit searching when the goal is reached
        # if n0.estimate(xB, yB) == 0:
        if x == xB and y == yB:
            # generate the path from finish to start
            # by following the dirs
            path = ''
            while not (x == xA and y == yA):
                j = dir_map[y][x]
                c = str((j + dirs / 2) % dirs)
                path = c + path
                x += dx[j]
                y += dy[j]
            return path

        # generate moves (child nodes) in all possible dirs
        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1
                    or the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
                # generate a child node
                m0 = node(xdx, ydy, n0.distance, n0.priority)
                m0.nextMove(dirs, i)
                m0.updatePriority(xB, yB)
                # if it is not in the open list then add into that
                if open_nodes_map[ydy][xdx] == 0:
                    open_nodes_map[ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    # mark its parent node direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                elif open_nodes_map[ydy][xdx] > m0.priority:
                    # update the priority
                    open_nodes_map[ydy][xdx] = m0.priority
                    # update the parent direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    # replace the node
                    # by emptying one pq to the other one
                    # except the node to be replaced will be ignored
                    # and the new node will be pushed in instead
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi]) # remove the target node
                    # empty the larger size priority queue to the smaller one
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])       
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0) # add the better node instead
    return '' # if no route found

# MAIN
dirs = 8 # number of possible directions to move on the map
if dirs == 4:
    dx = [1, 0, -1, 0]
    dy = [0, 1, 0, -1]
elif dirs == 8:
    dx = [1, 1, 0, -1, -1, -1, 0, 1]
    dy = [0, 1, 1, 1, 0, -1, -1, -1]

n = 6 # horizontal size of the map
m = 6 # vertical size of the map
the_map = matrix
print(the_map)
#row = [0] * n
#for i in range(m): # create empty map
   # the_map.append(list(row))
#print(the_map)
# fillout the map with a '+' pattern
#for x in range(n / 8, n * 7 / 8):
   # the_map[m / 2][x] = 1
#for y in range(m/8, m * 7 / 8):
   # the_map[y][n / 2] = 1

# randomly select start and finish locations from a list
sf = []
sf.append((0, 0, n - 1, m - 1))
#sf.append((0, m - 1, n - 1, 0))
#sf.append((n / 2 - 1, m / 2 - 1, n / 2 + 1, m / 2 + 1))
#sf.append((n / 2 - 1, m / 2 + 1, n / 2 + 1, m / 2 - 1))
#sf.append((n / 2 - 1, 0, n / 2 + 1, m - 1))
#sf.append((n / 2 + 1, m - 1, n / 2 - 1, 0))
#sf.append((0, m / 2 - 1, n - 1, m / 2 + 1))
#sf.append((n - 1, m / 2 + 1, 0, m / 2 - 1))
(xA, yA, xB, yB) = random.choice(sf)

print 'Map size (X,Y): ', n, m
print 'Start: ', xA, yA
print 'Finish: ', xB, yB
t = time.time()
route = pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)
print 'Time to generate the route (seconds): ', time.time() - t
print 'Route:'
print route

# mark the route on the map
if len(route) > 0:
    x = xA
    y = yA
    the_map[y][x] = 2
    for i in range(len(route)):
        j = int(route[i])
        x += dx[j]
        
        y += dy[j]
        the_map[y][x] = 3
        #print(y,x)
    the_map[y][x] = 4
    print(the_map)

# display the map with the route added
print 'Map:'
for y in range(m):
    for x in range(n):
        xy = the_map[y][x]
        if xy == 0:
            print '.', # space
        elif xy == 1:
            print 'O', # obstacle
        elif xy == 2:
            print 'S', # start
        elif xy == 3:
            print 'R', # route
            print(y,x)
            v=y-yA
            w=x-xA
            abcd(v,w)
            yA=y
            xA=x
        elif xy == 4:
            print 'F', # finish
            v=yB-yA
            w=xB-xA
            abcd(v,w)
    print

raw_input('Press Enter...')
