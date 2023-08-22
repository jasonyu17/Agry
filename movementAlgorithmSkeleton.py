def goAround():
    turn_left90()
    camera()
    forward(1,3)
    turn_right90()
    camera()
    forward(1,3)
    camera()
    forward(1,3)
    turn_right90()
    camera()
    forward(1,3)
    turn_left90()

def skipAroundLeft():
    turn_left90()
    camera()
    forward(1,3)
    turn_right90()
    camera()
    forward(1,3)
    turn_left90()
    turn_left90()

def skipAroundRight():
    turn_right90()
    camera()
    forward(1,3)
    turn_left90()
    camera()
    forward(1,3)
    turn_right90()
    turn_right90()



dirLeft = True
skippedAround = False

i = 0
while i < 4:
    j = 0
    while j < 4:
        drill()
        if(cam() == blocked):
            if (j==0 or j==1):
                goAround()
                j += 1
            elif(j == 2 and dirLeft):
                skipAroundLeft()
                skippedAround = True
                j += 1
            elif(j == 2 and dirLeft == False):
                skipAroundRight()
                skippedAround = True
                j += 1
        if (j != 3):
            forward()
        j += 1
    if (skippedAround == False):
        if(dirLeft):
            turn_left90()
            camera()
            forward(1,3)
            turn_left90()
            dirLeft = False
        else:
            turn_right90()
            camera()
            forward(1,3)
            turn_right90()
            dirLeft = True
    skippedAround = False
    i += 1
