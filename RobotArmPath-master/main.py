import math
import numpy as np
import matplotlib.pyplot as plt
import copy
import heapq

fixedPoint = [0,0]
# closedList = [list([list([False for i in range(360)]) for j in range(360)]) for k in range(360)]
closedList = [list([False for i in range(360)]) for j in range(360)]

def getAngle(Angle):
    # return closedList[Angle[0]][Angle[1]][Angle[2]]
    return closedList[Angle[0]][Angle[1]]

def setAngle(Angle):
    # closedList[Angle[0]][Angle[1]][Angle[2]] = True
    closedList[Angle[0]][Angle[1]] = True
    return

def euclidian_dist(p1,p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

def heuristic_fn(goalPoint,endPoint,arms):
    return math.degrees(euclidian_dist(goalPoint,endPoint)/sum(arms))

def cylindrical_Coordinates(coordinate,length,angle):
    res_Cor = [0 ,0]
    res_Cor[0] = coordinate[0] + length*math.cos(math.radians(angle))
    res_Cor[1] = coordinate[1] + length*math.sin(math.radians(angle))
    return res_Cor

def arm_endPoint(initial_Cor,lengths,angles):
    endPoint = initial_Cor
    angle = 0
    for i in range(len(lengths)):
        angle += angles[i]
        endPoint = cylindrical_Coordinates(endPoint,lengths[i],angle)
    endPoint = np.round_(endPoint,decimals=3)
    return endPoint

def bfs(arms,initial_angles,goalPoint,increment,error):
    openlist = []
    closedlist = []
    openlist.append(initial_angles)
    step = 0
    while len(openlist)!=0:
        step += 1
        currAngle = copy.deepcopy(openlist.pop(0))
        currAngle = list([num%360 for num in currAngle])
        # closedlist.append(list(currAngle))
        setAngle(currAngle)
        if euclidian_dist(arm_endPoint(fixedPoint,arms,currAngle),goalPoint) < error:
            return currAngle
        
        for i in range(len(currAngle)):
            currAngle[i] += increment
            currAngle = list([num%360 for num in currAngle])
            if not getAngle(currAngle):
                openlist.append(copy.deepcopy(currAngle))
            currAngle[i] -= 2*increment
            currAngle = list([num%360 for num in currAngle])
            if not getAngle(currAngle):
                openlist.append(copy.deepcopy(currAngle))
            currAngle[i] += increment
            currAngle = list([num%360 for num in currAngle])
        # if step%10 == 0:
        #     print(closedlist)
        # print(euclidian_dist(arm_endPoint(fixedPoint,arms,currAngle),goalPoint))

def Astar(arms,initial_angles,goalPoint,increment,error):
    openlist = []
    closedlist = []
    h_0 = heuristic_fn(goalPoint,arm_endPoint([0,0],arms,initial_angles),arms)
    heapq.heappush(openlist,(h_0,0,initial_angles))
    step = 0
    while len(openlist)!=0:
        step += 1
        hp = heapq.heappop(openlist)
        g_n = int(hp[1])
        currAngle = copy.deepcopy(hp[2])
        currAngle = list([num%360 for num in currAngle])
        # closedlist.append(list(currAngle))
        setAngle(currAngle)
        if euclidian_dist(arm_endPoint(fixedPoint,arms,currAngle),goalPoint) < error:
            return currAngle
        
        for i in range(len(currAngle)):
            currAngle[i] += increment
            currAngle = list([num%360 for num in currAngle])
            if not getAngle(currAngle):
                h_n = heuristic_fn(goalPoint,arm_endPoint([0,0],arms,currAngle),arms)
                heapq.heappush(openlist,(int(g_n+h_n+increment),g_n+increment,copy.deepcopy(currAngle)))
            currAngle[i] -= 2*increment
            currAngle = list([num%360 for num in currAngle])
            if not getAngle(currAngle):
                h_n = heuristic_fn(goalPoint,arm_endPoint([0,0],arms,currAngle),arms)
                heapq.heappush(openlist,(int(g_n+h_n+increment),g_n+increment,copy.deepcopy(currAngle)))
            currAngle[i] += increment
            currAngle = list([num%360 for num in currAngle])
        # print(h_n)
        # if step%10 == 0:
        #     print(closedlist)
        # print(euclidian_dist(arm_endPoint(fixedPoint,arms,currAngle),goalPoint))
    #     plt.xlim(0,360)
    #     plt.ylim(0,360)
    #     op = np.array(openlist)[:,2]
    #     plt.scatter([i[0] for i in op],[i[1] for i in op],s=10,color='b')
    #     plt.pause(0.1)
    # plt.show()
def main():
    arms = [1,1]
    initial_angles = [30,0]
    goalPoint = [-0.5,1]
    endPoint = arm_endPoint(fixedPoint,arms,initial_angles)
    print(endPoint)

    # searchGoal = bfs(arms,initial_angles,goalPoint,7,0.4)
    searchGoal = Astar(arms,initial_angles,goalPoint,7,0.4)
    print(searchGoal)
    print(arm_endPoint(fixedPoint,arms,searchGoal))


if __name__ == "__main__":
    main()