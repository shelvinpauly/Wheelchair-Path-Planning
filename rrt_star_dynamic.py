# -*- coding: utf-8 -*-
"""
RRT* with dynamic obs

@author: Sharmitha Ganesan
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import cv2
import heapq
import random
'''
Autonomous wheelchair navigation in airport

'''
#size 640,480 in cm
start = (100,100)
goal = (305,220)
step= 10

#lists used
all_points = []
static_points = []
clearance_points = []
dynamic_points = []
visited=set()
backtracking={}
x_pts=0
y_pts=0

for i in range(640):
    for j in range(480):
        x_pts = round(i)
        y_pts = round(j)
        all_points.append((x_pts,y_pts))


#static obstacle space which defines the pillars, seats in the airport

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
#DYNAMIC OBSTACLE:
def dynamic_in_out(x1,y1):
    dynamic_points=[]
    if y1 >=480:
        y1 = 480
    if x1>=640:
        x1 = 640
    
    cv2.circle(blank_canvas,(210,y1-10),5,(0,0,0),-1)
    cv2.circle(blank_canvas,(x1-10,100),5,(0,0,0),-1)
    
    for pt in all_points:
       
       x = pt[0]
       y = pt[1]  
                
       if (x-210)**2 + (y-y1)**2 <= (15)**2:
          dynamic_points.append(((x,y),(340,y1))) # returning circle points and its center
          
       if (x- x1)**2 + (y - 100)**2 <= (15)**2:
          dynamic_points.append(((x,y),(x1,100))) # returning circle points and its center
         
  
    cv2.circle(blank_canvas,(210,y1),5,(245,245,137),-1)
    cv2.circle(blank_canvas,(x1,100),5,(245,245,137),-1)
    
    return dynamic_points
#plotting

blank_canvas = np.zeros((480,640,3),np.uint8) 

        
for s in static_points: 
    x = int(s[1])
    y = int(s[0])
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

def Eucledian(a, b):
    
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
   
    dist = round(dist)
    return dist

def rand_pt_gen():
        while True:
           
            random1 = random.choice(all_points)
            if random1 not in static_points:
                return random1
def next_node(node,x1,y1):
    global rand_pt
    while True:
        theta = 0
        rand_pt = rand_pt_gen()
        cv2.circle(blank_canvas,rand_pt,2,[255,0,0],-1)
        print('randon',rand_pt)
        if rand_pt[0]-node[0] == 0:
            print('undefined')
            if rand_pt[1]>node[1]:
                
                theta = np.radians(90)
            else:
                if rand_pt[1]<node[1]:
                    
                    theta = np.radians(270)
        else:
            slope = (node[1]-rand_pt[1])/(node[0]-rand_pt[0])
            theta = math.atan(slope)
        if rand_pt[1]>node[1] and np.degrees(theta)<0 :
            print('wrong',np.degrees(theta))
            theta = 180 + np.degrees(theta)
            print('correct',theta)
            theta = np.radians(theta)
        if rand_pt[1]<node[1] and np.degrees(theta)>0 :
            print('wrong',np.degrees(theta))
            theta = np.degrees(theta) -180
            print('correct',theta)
            theta = np.radians(theta)
        print('THETA',np.degrees(theta))    
        new_x = node[0]+ round(step*math.cos(theta))
        new_y = node[1]+ round(step*math.sin(theta)) 
        
        if obs_in_out((new_x,new_y),x1,y1) ==True and new_x>=0 and new_x<=640 and new_y>=0 and new_y<=480:    
                return (new_x,new_y),True
        else:
            print('reject',(new_x,new_y))
            return node,False
        
def nearest_nodes_finder(new_node):
    neighbors=[]
    near_nodes=[]
    for pt in all_points:
        
          x = pt[0]
          y = pt[1]
          
          if (x - new_node[0])**2 + (y - new_node[1])**2 <=(20)**2:
              neighbors.append((x,y))
    for node in visited:
        if node in neighbors:
            near_nodes.append(node)
           
    return near_nodes

def cost_update(node,x1,y1):
   
    cost={}
    new_node,success = next_node(node,x1,y1)
    if success == True:
        local_cost = Eucledian(node,new_node)
        
        cost[new_node] = (local_cost)
        return cost
    else:
        local_cost = 10000
        
        cost[new_node] = (local_cost)
        return cost
#getting the storage arrays ready

cost_to_come = np.array(np.ones((641, 481)))
vis = np.array(np.zeros((641, 481)))
priority_queue = []
heapq.heappush(priority_queue, (0, start))

# initialize cost and vis for start node to zero
cost_to_come[int(start[0])][int(start[1])] = 0



def obs_in_out(point,x1,y1):
    dynamic= dynamic_in_out(x1, y1)
    
    if point in static_points or point in dynamic[0]:
        return False
    else:
        return True
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
                    if v2 not in back_track_list:
                        back_track_list.append(from_goal)
                        print('Backtrack',back_track_list)
                    # updating the start variable
                    from_goal = v2

                    # checking if it is the goal
                    if v2 == back_to_start:
                        back_to_start = []
                        
                        break
    back_track_list.append(from_goal)
    return (back_track_list)

def rrt_star_dynamic(start,goal):
    print('IN RRT STAR')
    visited.add(start)
    break_while=0
    x1 = 10
    y1 = 10
   
    if goal in static_points or start in static_points:

        print('GOAL/START is in obstacle space')
    
        return False

    else:
        while True:
           
            if break_while ==1:
                break
            print('In start of loop')
            
            start_iter = random.choice(tuple(visited))
            print('rand_iter',start_iter)
            itr = 0
           
            while True:
                if vis[int(start_iter[0])][int(start_iter[1])] ==5:
                    start_iter = random.choice(tuple(visited))
                    print('rand_iter_after_5repetition',start_iter)
                    itr = 0
                else:
                    break
                
            for itr in range(50):
                _,curr_vert = heapq.heappop(priority_queue)
                
                print('CURRENT',curr_vert)
                if itr ==0:
                    curr_vert = start_iter
                # append visited nodes
                vis[int(curr_vert[0])][int(curr_vert[1])] +=1
                visited.add(curr_vert)
                if ((curr_vert[0] - goal[0]) ** 2 + (curr_vert[1] - goal[1]) ** 2 <= (5) ** 2):
                    print('GOAL FOUND...YAY!!',curr_vert)
                    break_while =1
                    break
                
                cv2.circle(blank_canvas, curr_vert, 2, [0,255,0], -1)
                
                if obs_in_out(curr_vert,x1,y1) == True:
                    
                    x1+=5
                    y1+=5
                    next_cost = cost_update(curr_vert,x1,y1)
                    print('next_with_cost',next_cost)
                    for key,val in next_cost.items(): #key is node and val is cost from parent to that node
                        next_vert = key
                        
                        cost_now = val
                        neighbor_nodes = nearest_nodes_finder(key)
                                     
                        if len(neighbor_nodes) > 0:
                           temp = []
                          
                           for points in neighbor_nodes:
                               cost_now = Eucledian(points,next_vert)
                               total = cost_now + cost_to_come[int(points[0])][int(points[1])]
                               temp.append(total)
                             
                           
                           min_cost = min(temp)
                           node_index = temp.index(min_cost)
                           
                        elif len(neighbor_nodes) == 0:
                            neighbor_nodes = [(curr_vert)]
                            node_index = 0
                        cost_now = Eucledian(neighbor_nodes[node_index],next_vert)    
                        print('chosen parent',neighbor_nodes[node_index])
                        cost_to_come[int(next_vert[0])][int(next_vert[1])] = cost_now + cost_to_come[int(neighbor_nodes[node_index][0])][int(neighbor_nodes[node_index][1])]
                        print('next-node',next_vert,cost_to_come[int(next_vert[0])][int(next_vert[1])])
                        # push to the explored node queue
                        cv2.line(blank_canvas,neighbor_nodes[node_index],next_vert,[255,255,255],1)
                        rrt = np.flipud(blank_canvas)
                        cv2.imshow('rrt_star',rrt)
                        cv2.waitKey(1)
                        cv2.circle(blank_canvas,rand_pt,2,[0,0,0],-1)
                        heapq.heappush(priority_queue, (cost_to_come[int(next_vert[0])][int(next_vert[1])], (next_vert)))
                        backtracking[next_vert] = {}
                        # adding to the backtracking dictionary
                        backtracking[next_vert][cost_to_come[int(next_vert[0])][int(next_vert[1])]] = (neighbor_nodes[node_index])
                        print('end of one loop')
        return curr_vert, backtracking


goal_found, backtracking_dict= rrt_star_dynamic(start, goal)

backtracked_final = BackTrack(backtracking_dict, start, goal_found)

start_goal =  backtracked_final[::-1]
f1 = open('path.txt', 'w')
print('start saving')
for i in start_goal:
   f1.write(str(i)+'\n')
f1.close()

print('saved rpm')              
print('back',backtracked_final)               

for i in range(len(backtracked_final)-1):
    point1 = backtracked_final[i]
    x1 = point1[1]
    y1 = point1[0]
    point2 = backtracked_final[i+1]
    x2 = point2[1]
    y2 = point2[0]
    cv2.line(blank_canvas,(y1,x1),(y2,x2),(255,192,203),2,cv2.LINE_AA)
   
    #setting every backtracked pixel to white
back_tracked_path = np.flipud(blank_canvas) 

#showing the image
cv2.imshow('backtracked',back_tracked_path)

cv2.waitKey(0)
cv2.destroyAllWindows()
                