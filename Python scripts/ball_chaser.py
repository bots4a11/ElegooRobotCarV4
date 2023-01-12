### Copyright Michael@bots4all
#%% Load modules
from IPython import get_ipython
import numpy as np
import cv2 as cv
from urllib.request import urlopen
import socket
import sys
import json
import re
import time
import imutils

#%% Clear working space
get_ipython().magic('clear')
get_ipython().magic('reset -f')

#%% Capture image from camera
cv.namedWindow('Camera')
cv.moveWindow('Camera', 0, 0)
cmd_no = 0
def capture():
    global cmd_no
    cmd_no += 1
    print(str(cmd_no) + ': capture image', end = ': ')
    cam = urlopen('http://192.168.4.1/capture')
    img = cam.read()
    img = np.asarray(bytearray(img), dtype = 'uint8')
    img = cv.imdecode(img, cv.IMREAD_UNCHANGED)
    # Filter image by color
    mask = cv.medianBlur(img, 5)
    img_hsv = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
    lower = np.array([50, 70, 60], dtype="uint8") # 50, 70, 60
    upper = np.array([90, 255, 255], dtype="uint8") # 90, 200, 230
    mask = cv.inRange(img_hsv, lower, upper)
    # Detect contours
    mask = cv.erode(mask, None, iterations = 2)
    mask = cv.dilate(mask, None, iterations = 2)
    cont = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cont = imutils.grab_contours(cont)
    # Evaluate all contours
    yh = 491        # y-coordinate of line of horizon, contours above it are ignored
    ball = 0        # flag indicating a presence of a ball of the given color
    dist = None     # distance to the ball
    ang_rad = 0     # angle to the ball in rad
    ang_deg = 0     # angle to the ball in deg
    area = 0        # area of contour
    area_max = 20   # contours with area smaller than this will be ignored 
    ncont = len(cont)
    if ncont > 0:
        for n in range(ncont):
            # Find center and area of contour
            M = cv.moments(cont[n])
            _xc = int(M['m10']/M['m00'])
            _yc = 600 - int(M['m01']/M['m00'])  # make y = 0 at image bottom
            area = M['m00']
            # Find ball with largest area below line of horizon
            if _yc < yh and area > area_max:
                area_max = area
                ball = 1
                nc = n
                xc = _xc - 400    # make x axis go through image center
                yc = _yc
                center = (_xc, 600 - _yc)   # need only for plotting
    # Calculate distance and angle to the ball
    if ball:
        cv.drawContours(img, cont, nc, (0,0,255), 1)    # draw selected contour
        cv.circle(img, center, 1, (0,0,255), 2)         # draw center
        cv.putText(img, '(' + str(xc) + ', ' + str(yc) + ')', center, 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv.LINE_AA)
        dy = 4.31*(745.2 + yc)/(yh - yc)    # distance to ball along y
        if xc < 0: dy = dy*(1 - xc/1848)    # correction factor for negative xc
        dx = 0.00252*xc*dy                  # distance to ball along x
        dist = np.sqrt(dx**2 + dy**2)       # distance to ball
        ang_rad = np.arctan(dx/dy)          # angle to ball in rad
        ang_deg = round(ang_rad*180/np.pi)  # angle to ball in deg
        print('bd =', round(dist), 'ba =', ang_deg)
    else:
        print('no ball')
    cv.line(img, (400,0), (400,600), (0,0,255), 1)           # center line
    cv.line(img, (0,600 - yh), (800,600 - yh), (0,0,255), 1) # line of horizon
    cv.imshow('Camera', img)
    cv.waitKey(1)
    return ball, dist, ang_rad, ang_deg

#%% Send a command and receive a response
off = [0.007,  0.022,  0.091,  0.012, -0.011, -0.05]
def cmd(sock, do, what = '', where = '', at = ''):
    global cmd_no
    cmd_no += 1
    msg = {"H":str(cmd_no)} # dictionary
    if do == 'move':
        msg["N"] = 3
        what = ' car '
        if where == 'forward':
            msg["D1"] = 3
        elif where == 'back':
            msg["D1"] = 4
        elif where == 'left':
            msg["D1"] = 1
        elif where == 'right':
            msg["D1"] = 2
        msg["D2"] = at # at is speed here
        where = where + ' '
    elif do == 'set':
        msg.update({"N":4,"D1":at[0],"D2":at[1]})
        what = ' speed '
    elif do == 'stop':
        msg.update({"N":1,"D1":0,"D2":0,"D3":1})
        what = ' car'
    elif do == 'rotate':
        msg.update({"N":5,"D1":1,"D2":at}) # at is an angle here
        what = ' head'
        where = ' '
    elif do == 'measure':
        if what == 'distance':
            msg.update({"N":21,"D1":2})
        elif what == 'motion':
            msg["N"] = 6
        what = ' ' + what
    elif do == 'check':
        msg["N"] = 23
        what = ' off the ground'
    msg_json = json.dumps(msg)
    print(str(cmd_no) + ': ' + do + what + where + str(at), end = ': ')
    try:
        sock.send(msg_json.encode())
    except:
        print('Error: ', sys.exc_info()[0])
        sys.exit()
    while 1:
        res = sock.recv(1024).decode()
        if '_' in res:
            break
    res = re.search('_(.*)}', res).group(1)
    if res == 'ok' or res == 'true':
        res = 1
    elif res == 'false':
        res = 0
    elif msg.get("N") == 5:
        time.sleep(0.5)                     # give time to rotate head
    elif msg.get("N") == 21:
        res = round(int(res)*1.3, 1)        # UM distance with correction factor
    elif msg.get("N") == 6:
        res = res.split(",")
        res = [int(x)/16384 for x in res]   # convert to units of g
        res[2] = res[2] - 1                 # subtract 1G from az
        res = [round(res[i] - off[i], 4) for i in range(6)]
    else:
        res = int(res)
    print(res)
    return res

#%% Connect to car's WiFi
ip = "192.168.4.1"
port = 100
print('Connect to {0}:{1}'.format(ip, port))
car = socket.socket()
try:
    car.connect((ip, port))
except:
    print('Error: ', sys.exc_info()[0])
    sys.exit()
print('Connected!')

#%% Read first data from socket
print('Receive from {0}:{1}'.format(ip, port))
try:
    data = car.recv(1024).decode()
except:
    print('Error: ', sys.exc_info()[0])
    sys.exit()
print('Received: ', data)

#%% Find the ball
speed = 100         # car speed
ang_tol = 10        # tolerance for rotation angle
ang = [90, ang_tol, 180 - ang_tol] # head rotation angles
dist = [0, 0, 0]    # measured distances to obstacles at rotation angles
dist_min = 30       # min distance to obstacle (cm)
d180 = 90           # eq rotation distance for 180 deg turn
dturn = 60          # eq rotation distance for smaller than 180 deg turns
def find_ball():
    time.sleep(0.5)
    found = 0
    for n in range(2):
        if n == 1:
            if dist[1] > dist[2]:
                cmd(car, do = 'move', where = 'right', at = speed)
            else:
                cmd(car, do = 'move', where = 'left', at = speed)
            time.sleep(d180/speed)
            cmd(car, do = 'stop')
        for i in range(3):
            cmd(car, do = 'rotate', at = ang[i])
            dist[i] = cmd(car, do = 'measure', what = 'distance')
            ball, bd, ba_rad, ba_deg = capture()
            if ball:
                if ((i == 1 and ba_deg < -ang_tol) or
                    (i == 2 and ba_deg > +ang_tol)):
                    # Rotate head more precisely to ball angle to measure distances
                    um_ang = ang[i] - ba_deg
                    cmd(car, do = 'rotate', at = um_ang)
                    d = cmd(car, do = 'measure', what = 'distance')
                    ball, bd, ba_rad, ba_deg = capture()
                else:
                    um_ang = ang[i]
                    d = dist[i]
                if not ball: continue
                if d > dist_min:
                    found = 1
                    print('found ball: bdist =', round(bd,1), 'dist =', d)
                    cmd(car, do = 'rotate', at = 90)
                    steer_ang = 90 - um_ang + ba_deg
                    if steer_ang > ang_tol:
                        cmd(car, do = 'move', where = 'right', at = speed)
                    elif steer_ang < -ang_tol:
                        cmd(car, do = 'move', where = 'left', at = speed)
                    print('steering angle =', steer_ang)
                    time.sleep(dturn/speed*abs(steer_ang)/180)
                    cmd(car, do = 'stop')
                    time.sleep(0.5)
                    _, bd, ba_rad, ba_deg = capture()
                break
        if found:
            break
    if not found:
        cmd(car, do = 'rotate', at = 90)

#%% Track the ball
def track_ball():
    ball, bd, ba_rad, ba_deg = capture()
    if ball:
        # Calculate left and right wheel speeds to reach the ball
        r = bd/(2*np.sin(ba_rad))    # required turning radius
        if r > 0 and r <= 707:  # turn right
            s0 = 1.111
            ra = -17.7
            rb = 98.4
        else:    # turn left or go straight
            s0 = 0.9557  # vl/vr speed ratio to go straight
            ra = 5.86
            rb = -55.9
        speed_ratio = s0*(r - ra)/(r + rb)
        speed_ratio = max(0, speed_ratio)
        if r > 0 and r <= 707:  # turn right
            lspeed = speed
            rspeed = round(speed*speed_ratio)
        else:                   # turn left or go straight
            lspeed = round(speed*speed_ratio)
            rspeed = speed
        cmd(car, do = 'set', at = [rspeed, lspeed])

#%% Main
cmd(car, do = 'rotate', at = 90)
find_ball()
while 1:
    # Check if car was lifted off the ground to interrupt the while loop
    if cmd(car, do = 'check'):
        break
    # Track the ball
    track_ball()
    # Check distance to obstacle
    if cmd(car, do = 'measure', what = 'distance') <= dist_min:
        # Detected an obstacle, stop
        cmd(car, do = 'stop')
        # Find the ball
        find_ball()

#%% Close socket
car.close()