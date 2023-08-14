
'''
PROJECT 5
a_star implementation
 @author Sharmitha Ganesan
'''

import matplotlib.pyplot as plt
import numpy as np
import math
import cv2
import numpy as np
import heapq


start = (100,100)
goal = (305,220)
theta = 0
L = 16 #bot_dia in cm
r = 3.3 #wheel rad in cm


#initial RPMs are zero
first = (start,theta)


#lists used
all_points = []
static_points = []
clearance_points = []
all_nodes = []
local_nodes = []
backtrack_dict={}
x_pts = []
y_pts = []


for i in range(640):
    for j in range(480):
        x_pts= round(i)
        y_pts = round(j)
        all_points.append((i,j))


#obs_space


for pt in all_points:
    
      x = pt[0]
      y = pt[1]
    
    #circle shaped obstacle with 10cm clearance
      if((x-100)**2 + (y-160)**2 <= (15)**2):
          static_points.append((x,y))

    #circle shaped obstacle with 10cm clearance
      if((x-540)**2 + (y-160)**2 <= (15)**2):
          static_points.append((x,y))
    #circle shaped obstacle with 10cm clearance
      if((x-540)**2 + (y-320)**2 <= (15)**2):
          static_points.append((x,y))

    #circle shaped obstacle with 10cm clearance
      if((x-100)**2 + (y-320)**2 <= (15)**2):
          static_points.append((x,y))
           
    #square obstacle with 10cm clearance
      if x>=240 and x<=280 and y>=200 and y<=280 :
          static_points.append((x,y))
   
      if x>=360 and x<=400 and y>=200 and y<=280 :
            static_points.append((x,y))
 
      
    #highlighting clearance
    #circle shaped obstacle with 10cm clearance
      if ((x-100)**2 + (y-160)**2 <= (10)**2):
          clearance_points.append((x,y))
          
    #circle shaped obstacle with 10cm clearance
      if((x-540)**2 + (y-160)**2 <= (10)**2):
          clearance_points.append((x,y))
    #circle shaped obstacle with 10cm clearance
      if((x-540)**2 + (y-320)**2 <= (10)**2):
          clearance_points.append((x,y))

    #circle shaped obstacle with 10cm clearance
      if((x-100)**2 + (y-320)**2 <= (10)**2):
          clearance_points.append((x,y))
           
    #square obstacle with 10cm clearance
      if x>=250 and x<=270 and y>=210 and y<=270 :
          clearance_points.append((x,y))
     #square obstacle with 10cm clearance
      if x>=370 and x<=390 and y>=210 and y<=270 :
           clearance_points.append((x,y))


blank_canvas = np.zeros((480,640,3),np.uint8) 

for l in static_points: 
    x = int(l[1])
    y = int(l[0])
    blank_canvas[(x,y)]=[0,0,255] #assigning a red coloured pixel
for d in clearance_points:
    x =int(d[1])
    y =int(d[0])
    blank_canvas[(x,y)] = [0,255,255]   #assigning a yellow coloured pixel
    
#flipping the image for correct orientation
obs_canvas = np.flipud(blank_canvas)
#showing the obstacle map
cv2.imshow('obstacle with clearance',obs_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()

def next_node_calc(old,UL,UR):
    
    
    r = 3.3
    L = 16
    
    theta_rad = 3.14 * (old[1]/180)
    dt = 0.1
    t = 0
    new_x = old[0][0]
    new_y = old[0][1]
    the = 0
    weight = 0
    UL = 0.1047 * UL
    UR = 0.1047 * UR
    while t<6:

        t= t+dt
        new_x += 0.5*r * (UL + UR) * math.cos(theta_rad) * dt
        new_y += 0.5*r * (UL + UR) * math.sin(theta_rad) * dt
        the += (r / L) * (UR - UL) * dt
        theta_new = int(180 * the / 3.14)
        if theta_new >=360:
            theta_new -=360
        if theta_new <=-360:
            theta_new +=360
         
        weight +=abs(math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(theta_rad) * dt),2)+ math.pow((0.5*r * (UL + UR) * math.sin(theta_rad) * dt),2)))
        new = [(round(new_x),round(new_y)),theta_new,weight]
        same = [old[0],int(old[1]),10000]
        cv2.circle(blank_canvas,new[0],0,(0,255,0),-1)  
    if new[0][0]>=0 and new[0][0]<=640.00 and new[0][1]>=0.00 and new[0][1]<=480.00 :
            return new,True
    else:
        return same,False

def graph(node):
       
        if node[0][0]>=0 and node[0][0]<=640.00 and node[0][1]>=0.00 and node[0][1]<=480.00:

            eight_locals = {}
            rpm1 = [5,10,0,0,5,5,10,20]
            rpm2 = [5,10,5,10,10,0,0,10]
            
            straight_near,_= next_node_calc(node,rpm1[0],rpm2[0])
            straight_far,_= next_node_calc(node,rpm1[1],rpm2[1])
            left_near,_= next_node_calc(node,rpm1[2],rpm2[2])
            left_far,_ = next_node_calc(node,rpm1[3],rpm2[3])
            left_diff ,_ = next_node_calc(node,rpm1[4],rpm2[4])
            right_near ,_ = next_node_calc(node,rpm1[5],rpm2[5])
            right_far ,_= next_node_calc(node,rpm1[6],rpm2[6])
            right_diff ,_= next_node_calc(node,rpm1[7],rpm2[7])
            if 0<= straight_near[0][0] <=640  and 0<= straight_near[0][1] <= 480:
                    eight_locals[straight_near[0]] = (round(straight_near[2], 2), straight_near[1])

            if 0<= straight_far[0][0] <=640  and 0<= straight_far[0][1] <= 480:
                eight_locals[straight_far[0]] = (round(straight_far[2], 2), straight_far[1])

            if 0<= left_near[0][0] <=640  and 0<= left_near[0][1] <= 480:
                eight_locals[left_near[0]] = (round(left_near[2], 2), left_near[1])

            if 0<= left_far[0][0] <=640  and 0<= left_near[0][1] <= 480:
                eight_locals[left_far[0]] = (round(left_far[2], 2), left_far[1])

            if 0<= left_diff[0][0] <=640  and 0<= left_diff[0][1] <= 480:
                eight_locals[left_diff[0]] = (round(left_diff[2], 2), left_diff[1])

            if 0<= right_near[0][0] <=640  and 0<= right_near[0][1] <= 480:
                eight_locals[right_near[0]] = (round(right_near[2], 2), right_near[1])

            if 0<= right_far[0][0] <=640  and 0<= right_near[0][1] <= 480:
                eight_locals[right_far[0]] = (round(right_far[2], 2), right_far[1])

            if 0<= right_diff[0][0] <=640  and 0<= right_diff[0][1] <= 480:
                eight_locals[right_diff[0]] = (round(right_diff[2], 2), right_diff[1])
            
            return eight_locals
        else:
            pass

def BackTrack(backtrack_dict, back_to_start, from_goal): 
   
    # initializing the backtracked list
    back_track_list = []
    
    while back_to_start != []:
        
        # for key and values in the backtracking dictionary
        for k, v in backtrack_dict.items():

            # for the key and values in the values, v
            for k2, v2 in v.items():
                # checking if the first key is the start
                if k == from_goal:
                    if v2[0] not in back_track_list:
                        back_track_list.append(from_goal)
                        
                    # updating the start variable
                    from_goal = v2[0]

                    # checking if it is the goal
                    if v2[0] == back_to_start:
                        back_to_start = []
                        
                        break
    back_track_list.append(from_goal)
    return (back_track_list)

backtracking = {}
# list of all the visited nodes
visited = set()

cost_to_come = np.array(np.ones((641, 481, 360)) * np.inf)
# Initializing visited nodes as empty array
V = np.zeros((641, 481, 360))
# array to store Heuristic distance
heur = np.array(np.ones((641, 481)) * np.inf)
# array to store total cost
total = np.array(np.ones((641, 481, 360)) * np.inf)
# list for Explored nodes
priority_queue = []
# append start point,start orientation of the bot and initialize it's cost to zero
heapq.heappush(priority_queue, (0, first))
# initialize cost  for start node to zero
cost_to_come[int(start[0])][int(start[1])][theta] = 0
total[int(start[0])][int(start[1])][theta] = 0

def obs_in_out(points):
    points = points[0]
    if points in static_points:
        return False
    else:
        return True

def Eucledian(a, b):
    
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
   
    dist = round(dist)
    return dist

##A* implementation

def a_star(first, goal):
    print('in astar')
    global static_points
    global V
    break_while = 0
    explored =[]
    backtracking = {}
    if goal in static_points or first[0] in static_points:

        print('GOAL/START is in obstacle space OR clearance OR might be inside the radius of bot')
        backtracking = 0
        rounded =0

        return False

    else:
        while True:
            
            if break_while == 1:
                break
            
            _,curr_vert = heapq.heappop(priority_queue)
            old_angle = int(curr_vert[1])
                        
            # append visited nodes
            visited.add(curr_vert)
            #cv2.circle(blank_canvas,curr_vert[0],2,(245,245,137),-1)
            explored_canvas = np.flipud(blank_canvas)
            cv2.imshow('explored ',explored_canvas)
            cv2.waitKey(1)
            #print('visited',visited)
            
            
            if ((curr_vert[0][0] - goal[0]) ** 2 + (curr_vert[0][1] - goal[1]) ** 2 <= (5) ** 2):
                print('GOAL FOUND...YAY!!',curr_vert)
                
                break

            # check whether node is in the obstacle space
            if obs_in_out(curr_vert) == True:
                
                eight_locals = graph(curr_vert)
                graph_list = []
                #print(eight_locals)
                for key, value in eight_locals.items():
                    graph_list.append((key, value))  #invidual access

                for k, v in graph_list:

                    cost_now = eight_locals[k][0]
                    breakflag = 0
                    angle = int(eight_locals[k][1])
                    # Round the node
                    rounded = ((round(k[0]), round(k[1])),angle)
                    explored.append(rounded[0])
                    if rounded in visited:
                        
                        breakflag = 1
                        # exit if found
                    if breakflag == 1:
                        continue
                    # check if this neighbour is goal
                    if (rounded[0][0] - goal[0]) ** 2 + (rounded[0][1] - goal[1]) ** 2 <= (5) ** 2:
                        print('GOAL FOUND...YAY!!',rounded[0])
                       
                        break_while = 1
                        
                    ##check whether current neighbour node is in the obstacle space
                    if obs_in_out(rounded) == True:
                        # check if visited
                        if V[int(rounded[0][0])][int(rounded[0][1])][angle] == 0:
                            # if not, make it visited
                            V[int(rounded[0][0])][int(rounded[0][1])][angle]= 1
                           
                            # calculate cost to come
                            
                            cost_to_come[int(rounded[0][0])][int(rounded[0][1])][angle] = (cost_now + cost_to_come[int(curr_vert[0][0])][int(curr_vert[0][1])][old_angle])
                    
                            # calculate cost to go values
                            heur[int(rounded[0][0])][int(rounded[0][1])] = Eucledian(rounded[0], goal)

                            # calculate total cost 
                            total[int(rounded[0][0])][int(rounded[0][1])][angle] = cost_to_come[int( rounded[0][0])][int(rounded[0][1])][angle] + heur[int(rounded[0][0])][int(rounded[0][1])]
                            
                            # push to the explored node queue
                            print('explored',(total[int(rounded[0][0])][int(rounded[0][1])][angle], (rounded)))
                            
                            heapq.heappush(priority_queue, (total[int(rounded[0][0])][int(rounded[0][1])][angle], (rounded)))
                            backtracking[rounded[0]] = {}
                            # adding to the backtracking dictionary
                            backtracking[rounded[0]][total[int(rounded[0][0])][int(rounded[0][1])][angle]] = (curr_vert)
                        else:
                            
                            if (total[int(rounded[0][0])][int(rounded[0][1])][angle]) > (total[int(curr_vert[0][0])][int(curr_vert[0][1])][old_angle]):
                                total[int(rounded[0][0])][int(rounded[0][1])][angle] = (total[int(curr_vert[0][0])][int(curr_vert[0][1])][old_angle])
                                backtracking[rounded[0]][total[int(rounded[0][0])][int(rounded[0][1])][angle]] = (curr_vert)
                    

    return curr_vert, backtracking,explored
    

new_goal_rounded, backtracking_dict,explored = a_star(first, goal)
# Backtracking back to the goal
backtracked_final = BackTrack(backtracking_dict, start, new_goal_rounded[0])
# printing the nodes from initial to the end
print(backtracked_final)




#backtracked path
for path in backtracked_final:
    
    #print(path)
    x = int(path[1])
    y = int(path[0])
    blank_canvas[(x,y)]=[0,255,0]
for i in range(len(backtracked_final)-1):
    x1 = backtracked_final[i][1]
    y1 = backtracked_final[i][0]
    x2 = backtracked_final[i+1][1]
    y2 = backtracked_final[i+1][0]
    cv2.line(blank_canvas,(y1,x1),(y2,x2),(255,255,255),2,cv2.LINE_AA)
   
    #setting every backtracked pixel to white
back_tracked_path = np.flipud(blank_canvas) 
#showing the image
cv2.imshow('backtracked',back_tracked_path)

cv2.waitKey(0)
cv2.destroyAllWindows()


