# Assesment-3
Design, implement, and document the software for autonomous mobile robot navigation in simulation
### The inputs required from the user
- ini_x : The initial position (x) of the robot.  
- ini_y : The initial position (y) of the robot.
- ini_A : The initial Angle of the robot.
- target_x_pos : The (x) position of the target.
- target_y_pos : The (y) position of the target.
### The Outputs from the program
- The distance between the robot and the target.
- The angle between the robot and the target.
- The coordinates of the obstcales
- A map containing the robot,the obstacles and the target showing the robot avoiding the obstacles to reach the target.
### Software versions
- Ubuntu : 18.4
- Python : 3.6.9
- Vs Code : 1.52.1
### The code function
The code is doing a simple simulation of a robot placed on a map at a specific position chosen by the user and the robot should reach the target chosen by the user, on his way there will be some obstacles generated randomly, and the robot should avoid these obstacles to reach the target.
### Thought Process
The robot can approach obstcales from different angles, so the robot should be programmed well to avoid these obstcales according to the angle. The following image shows the different possible angles a robot might approache an obstcale and the angles that should steer with to avoid it.![Conditions](https://user-images.githubusercontent.com/114658806/206795097-6c044274-c319-4efa-82c3-e34217e6986e.jpeg)
### Flowchart
![Flowchart](https://user-images.githubusercontent.com/114658806/206788126-12ceeafd-8354-43d3-8160-332a453a2f34.png)
### The Results
The robot reaches the target while avoiding any obstacle in its path
![robot](https://user-images.githubusercontent.com/114658806/206789606-f23f01bf-d76f-4998-b646-8062f5759c76.png)
### Code
```
#Importing libraries 
from roboticstoolbox import Bicycle, RandomPath, VehicleIcon, RangeBearingSensor, LandmarkMap
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from math import atan2, pi, cos, sin, sqrt
from cmath import sqrt


#Asking the user to input x position of the robot
while(True):
    ini_x = float(input('Enter initial x position of the robot between -20 and 20: \n'))
    if((ini_x > 20) or (ini_x < -20)):
        print('please enter a number between -20 and 20 \n')
    else:
        break       #To exit the while loop


#Asking the user to input Y position of the robot
while(True):
    ini_y = float(input('Enter initial Y position of the robot between -20 and 20: \n'))
    if((ini_y > 20) or (ini_y < -20)):
        print('please enter a number between -20 and 20 \n')
    else:
        break      #To exit the while loop


#Asking the user to input the angle of the robot
ini_A = float(input('Enter initial angle of the robot: \n'))


#Asking the user to set the X target
while(True):
    target_x_pos = float(input('Enter X position of the target between -20 and 20: \n'))
    if((target_x_pos > 20) or (target_x_pos < -20)):
        print('please enter a number between -20 and 20 \n')
    else:
        break         #To exit the while loop


#Asking the user to set the Y target
while(True):
    target_y_pos = float(input('Enter Y position of the target between -20 and 20: \n'))
    if((target_y_pos > 20) or (target_y_pos < -20)):
        print('please enter a number between -20 and 20 \n')
    else:
        break         #To exit the while loop


#Adding the vehicle icon and its scale
anim = VehicleIcon('robot',scale = 3)
veh = Bicycle(
    animation = anim,
    control = RandomPath,
    dim = 10,
    x0 = (ini_x, ini_y, ini_A)          #inital position
)
veh.init(plot=True)


#Create the target point
target = [target_x_pos, target_y_pos];
goal_marker_style = {
    'marker': 'D',
    'markersize': 6,
    'color': 'b',
}
plt.plot(target[0], target[1], **goal_marker_style)



#Loading LandmarkMap to add the obstacles randomly
map = LandmarkMap(20, 20)
map.plot()
image = mpimg.imread('map.png')    #Loading A map
plt.imshow(image, extent = [-20,20,-20,20])


#Adding RangeBearingSensor 
sensor = RangeBearingSensor(robot=veh,map=map,animate=True)


#Reading distance between robot and obstacles using RangeBearingSensor
def read_distances(RD):
    distances = [i[0] for i in RD]
    return distances


#Reading angles between robot and obstacles using RangeBearingSensor
def read_angles(RA):
    angles =[i[1]*(180/pi) for i in RA]      #Converting the angle into radians
    return angles


#Finding coordinates of the obstacles to be printed after the robot reaches the target
def coordinates(x, y):
    x_coor = [x[0] + (i[0]*cos(i[1])) for i in y]
    y_coor = [x[1] + (i[0]*sin(i[1])) for i in y]
    final_coor = [x_coor + y_coor]
    print('Coordinates of the obstacles: \n {}' .format(final_coor))
    return final_coor


#Calculate the distance and the angle between robot and target
def measurements(x, y):
    distance = sqrt(abs(y[0] - x[0])**2 + (abs(y[1] - x[1]))**2)     
    angle = atan2((abs(y[1] - x[1])), (abs(y[0] - x[0])))
    print('Distancee between Robot and Target = : {}'.format(distance))
    print('Angle between Robot and Target = : {} \n'.format(angle))


#Steering to the target
def steering(x, y):
    goal_heading = atan2(
    y[1] - x[1],
    y[0] - x[0]
    )
    Steer = goal_heading-veh.x[2]
    return Steer

#Define a function to avoid obstacles
def detect_obstacles(run):
    steer = steering(veh.x ,target)
    for i in sensor.h(veh.x):
        if(i[0] < 3.5):             #4 is a tolerance
            step = True
            while(step):
                for j in sensor.h(veh.x):     #Some conditions to avoid obstacles according to the angle
                    #First condition
                    if ( (j[1] > pi/4) and (j[1] < pi/2) ):
                        veh.step(0.125, pi/4 + j[1])
                        step = False
                    #Second condition
                    elif ( (j[1] < pi/4) and (j[1] > 0) ):
                        veh.step(0.125, -pi/4 + j[1])
                        step = False
                    #Third condition
                    elif( (j[1] < (3*(pi/4))) and (j[1] > pi/2) ):
                        veh.step(0.125, -pi/4 + j[1])
                        step = False
                    #Fourth condition
                    elif( (j[1] > (3*(pi/4))) and (j[1] < pi) ):
                        veh.step(0.125, pi/4 + j[1])
                        step = False
                    #Fifth condition
                    elif( (j[1] < -pi/2) and (j[1] > (-3*(pi/4))) ):
                        veh.step(0.125, pi/4 + j[1])
                        step = False
                    #Sixth condition
                    elif( (j[1] < -(3*(pi/4))) and (j[1] > -pi) ):
                        veh.step(0.125, -pi/4 + j[0])
                        step = False
                    #Seventh condition
                    elif( (j[1] > -pi/4) and (j[1] < 0) ):
                        veh.step(0.125, pi/4 + j[1])
                        step = False
                    #Eighth condition
                    elif( (j[1] < -pi/4) and (j[1] > -pi/2)):
                        veh.step(0.125, -pi/4 + j[1])
                        step = False

                veh._animation.update(veh.x)
                plt.pause(0.005)
                break                           #To exit the for loop

        else:
            veh.step(0.2,steer)
            veh._animation.update(veh.x)
    plt.pause(0.005)
    return run

run = True      
while(run):
    steer = steering(veh.x, target)
    if( (abs(target[0]-veh.x[0]) > 0.6) or (abs(target[1]-veh.x[1]) >0.6) ):     #0.6 is a tolerance to stop the robot when reaches its target
        run = True
    else:
        run = False
    detect_obstacles(run)
    measurements(veh.x, target)

coordinates([ini_x, ini_y], sensor.h(veh.x))

plt.pause(100)
print(veh.x)
```
### Further Improvement
- The robot needs to identify if there is any obstacle at the target point.
- The robot needs to avoid more than one obstacle at the same time.
- The code needs to be improved if there is any obstacle at the starting point.
- The code needs to be improved to walk between the walls.
- The code needs to be improved to draw a line from the starting point avoiding the obstacles to the target point and the robot walk on it.
- Simplify the eight conditions  
