#!/usr/bin/env pybricks-micropython
from math import floor, ceil
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

timer = StopWatch()

left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
# mid_motor = Motor(Port.A)

LS_cube = ColorSensor(Port.S1)
LS_right = ColorSensor(Port.S2)
LS_left = ColorSensor(Port.S3)
LS_center = ColorSensor(Port.S4)
# LS_cube = ColorSensor(Port.S4)

# Button1 = TouchSensor(Port.S1)

# LS_side_right = ColorSensor(Port.S4)
# LS_side_left = ColorSensor(Port.S1)

# US = UltrasonicSensor(Port.S4)

# if LS_right.color() == Color.GREEN
S = 0
d = 62.4
deg_enc = left_motor.angle()

number = 0

error_i = 0
last_error = 0

filtered = [0, 0, 0]

robot = DriveBase(left_motor, right_motor, wheel_diameter=64, axle_track=104)


graph = {"a": ["0", "F", "b", "c"],
         "b": ["a", "d", "e", "e"],
         "c": ["1", "a", "e", "2"],
         "d": ["b", "6", "f", "f"],
         "e": ["b", "b", "g", "c"],
         "f": ["d", "5", "g", "d"],
         "g": ["e", "f", "4", "3"]}
graph_dir = {"a": ["00", "F1", "b2", "c3"],
         "b": ["a0", "d2", "e3", "e2"],
         "c": ["10", "a1", "e1", "23"],
         "d": ["b3", "61", "f2", "f1"],
         "e": ["b1", "b0", "g2", "c0"],
         "f": ["d0", "51", "g3", "d0"],
         "g": ["e0", "f0", "42", "32"]}
directions = {"F": "3", "0": "2", "1": "2", "2": "1", "3": "1", "4": "0", "5": "3", "6": "3"}
# graph in format ({cross_name: [up, left, down, right]})

# graph = {"a": ["0", "F", "b", "c"],
#          "b": ["a", "d", "e", "e"],
#          "c": ["1", "a", "e", "2"],
#          "d": ["b", "6", "f", "f"],
#          "e": ["b", "b", "g", "c"],
#          "f": ["d", "d", "5", "g"],
#          "g": ["e", "f", "4", "3"]}
# graph_dir = {"a": ["00", "F1", "b2", "c3"],
#          "b": ["a0", "d2", "e3", "e2"],
#          "c": ["10", "a1", "e1", "23"],
#          "d": ["b3", "61", "f3", "f2"],
#          "e": ["b1", "b0", "g2", "c0"],
#          "f": ["d1", "d0", "51", "g3"],
#          "g": ["e0", "f1", "42", "32"]}
# # direction at end 0-up, 1-left, 2-down, 3-right
#
# directions = {"F": "3", "0": "2", "1": "2", "2": "1", "3": "1", "4": "0", "5": "0", "6": "3"}
from_punkt = {"F": "a", "0": "a", "1": "c", "2": "c", "3": "g", "4": "g", "5": "f", "6": "d"}
straight_paths = ["0abec1", "Fac2", "3gfdbeg4", "5fd6"]


# функции
def mm_to_deg(mm):
    global d
    return mm * 360 / d / 3.1415


def deg_to_mm(deg):
    global d
    return deg / 360 * d * 3.1415


def motors_stop():
    left_motor.stop()
    right_motor.stop()

def pid_reg(v,kp,ki,kd,n,ride=500):
    error_i = 0
    last_error = 0
    for _ in range(n):
        while LS_right.reflection() > 40 or LS_left.reflection() > 40:
            
            error = LS_right.reflection() - LS_left.reflection()
            error_i = error + error_i
            error_d = error - last_error

            p = kp * error
            i = ki * error_i
            d = kd * error_d

            u = p + i + d
            
            last_error = error

            left_motor.run(v + u)
            right_motor.run(v - u)
            
            wait(10)
        if ride:
            left_motor.run(300)
            right_motor.run(300)
            wait(ride)
        left_motor.stop()
        right_motor.stop()


def pid_reg_deg(v,kp,ki,kd,deg):
    old_deg = left_motor.angle()
    error_i = 0
    last_error = 0
    while left_motor.angle() - old_deg < deg:
        
        error = LS_right.reflection() - LS_left.reflection()
        error_i = error + error_i
        error_d = error - last_error

        p = kp * error
        i = ki * error_i
        d = kd * error_d

        u = p + i + d
        
        last_error = error

        left_motor.run(v + u)
        right_motor.run(v - u)
        
        wait(10)
    left_motor.stop()
    right_motor.stop()


def pid_reg2(v,kp,ki,kd,n):
    error_i = 0
    last_error = 0
    for _ in range(n):
        while LS_right.reflection() > 40 or LS_left.reflection() > 40:
            
            error = (LS_side_right.reflection() + LS_right.reflection()) - (LS_left.reflection() + LS_side_left.reflection())
            # if abs(error_i) > abs(error + error_i):
                # error_i = error * 2 + error_i
            # else:
            error_i = error + error_i
            error_d = error - last_error

            p = kp * error
            i = ki * error_i
            d = kd * error_d

            u = p + i + d
            
            last_error = error

            left_motor.run(v + u)
            right_motor.run(v - u)
            
            wait(10)
        left_motor.run(300)
        right_motor.run(300)
        wait(100)
        left_motor.stop()
        right_motor.stop()

def empty_pid_reg(v,kp,ki,kd, offset = 0):
        global error_i, last_error
        error = LS_right.reflection() - LS_left.reflection()
        error_i = error + error_i
        error_d = error - last_error

        p = kp * error
        i = ki * error_i
        d = kd * error_d

        u = p + i + d
        
        last_error = error

        left_motor.run(v + u)
        right_motor.run(v - u + offset)
        
        wait(10)


def filter(data, n):
    global filtered
    filtered.append(data)
    if len(filtered) >= 2*n:
        filtered = filtered[-1*n:]
        # ev3.screen.print(filtered[:n])
    return sorted(filtered[:n])[n//2]


def enter():
    global number
    while not Button.CENTER in ev3.buttons.pressed():
        if Button.UP in ev3.buttons.pressed():
            number += 1
            wait(200)
        if Button.DOWN in ev3.buttons.pressed():
            number -= 1
            wait(200)
        if Button.RIGHT in ev3.buttons.pressed():
            number += 10
        if Button.LEFT in ev3.buttons.pressed():
            number -= 10
            wait(50)
        if Button1.pressed():
            number = number * 10
            wait(400)
        ev3.screen.clear()
        ev3.screen.print(number)
        wait(10)


def zone(n, x_0):
    global deg_enc, d
    pi_reg(300, 0.3, 0, 0, 1)
    S = (left_motor.angle() - deg_enc) * 3.1415 * d / 360
    return floor(3 - ((S - x_0) / 300))


def zone2(n, y_0):
    # pi_reg(300, 0.3, 0, 0, 1)
    y_sum = 210
    S = US.distance()
    return ceil((S - y_0) / (y_sum / n))


def zone3(n, b_0):
    global deg_enc, d
    error_i = 0
    while US.distance() >= 200:
        
        error = LS_right.reflection() - LS_left.reflection()
        error_i = error + error_i

        p = 0.3 * error
        i = 0.003 * error_i

        turn_rate = p + i
        
        last_error = error

        robot.drive(300, turn_rate)
    robot.stop()
    
    deg_enc = left_motor.angle()

    error_i = 0
    while US.distance() < 200:
        
        error = LS_right.reflection() - LS_left.reflection()
        error_i = error + error_i

        p = 0.3 * error
        i = 0.003 * error_i

        turn_rate = p + i
        
        last_error = error

        robot.drive(300, turn_rate)
    robot.stop()

    S = (left_motor.angle() - deg_enc) * 3.1415 * d / 360
    
    return (S > 150) + 1


def tank_turn(degrees):
    global d
    deg = left_motor.angle()
    while abs(left_motor.angle() - deg) < abs(degrees) * 180 / d:
        left_motor.run(300 * degrees // abs(degrees))
        right_motor.run(-300 * degrees // abs(degrees))
    left_motor.stop()
    right_motor.stop()


def turn(n, v, t, back=0):
    for _ in range(n):
        left_motor.run(v)
        right_motor.run(-v)
        wait(t)
        while ((LS_right.reflection() * (v > 0)) + (LS_left.reflection() * (v < 0))) > 16:
            left_motor.run(v)
            right_motor.run(-v)
        left_motor.run(-v)
        right_motor.run(v)
        wait(back)
        motors_stop()


def anti_turn(n, v, t):
    for _ in range(n):
        left_motor.run(v)
        right_motor.run(-v)
        wait(t)
        while (LS_left.reflection() * (v > 0)) + (LS_right.reflection() * (v < 0)) > 20:
            left_motor.run(v)
            right_motor.run(-v)
        motors_stop()


def finish():
    pi_reg(300, 0.3, 0.003, 0, 1)    
    robot.straight(150)
    tank_turn(90 * (number - 2))


def ride_mm(v, mm):
    global d
    deg = left_motor.angle()
    while abs(deg - left_motor.angle()) < mm * 360 / d / 3.1415:
        left_motor.run(v)
        right_motor.run(v)
    left_motor.stop()
    right_motor.stop()


def to_first():
    ride_mm(300, 100)
    pid_reg(300, 2, 0, 0, 1)
    ride_mm(300, 20)
    tank_turn(-90)
    pid_reg_deg(300, 2, 0, 0, mm_to_deg(180))


def open():
    mid_motor.run_until_stalled(300, duty_limit=50)


def close():
    mid_motor.run_until_stalled(-300, duty_limit=50)
    mid_motor.hold()


def tree_path(start, end, path=[], min=10):
    global graph, from_punkt
    if len(path)==1:path=[]
    if start==end: return [path, len(path)]
    if len(path) <= min:
        if start != "F":
            if start.isnumeric():
                start=from_punkt.get(start)
                path.append(start)
        else:
            start=from_punkt.get(start)
            path.append(start)
        best_path=None
        for i in graph.get(start):
            if (not i.isnumeric() and i != "F") or i == end:
                path2=path.copy()
                path2.append(i)
                result=tree_path(i, end, path2, min)
                if result:
                    if len(result[0]) < min:
                        min = len(result[0])
                        best_path = result[0]
        if best_path:
            return [best_path, min]


def pathfinding(start_, end):
    global graph, graph_dir, directions, from_punkt, straight_paths
    # start_ = input("start:\n")
    # end = input("end:\n")

    path = [start_] + tree_path(start_, end)[0]

    algorithm = []

    # print(path)

    direction = int(directions.get(start_))

    # print(direction)
    # print(path)
    for i in range(1, len(path)-1):
        # print(graph_dir.get(path[i]))
        for i2 in range(len(graph_dir.get(path[i]))):
            if graph_dir.get(path[i])[i2][0] == path[i+1]:
                current_dir = i2
                next_dir = graph_dir.get(path[i])[i2][1]

                break

        # print(current_dir, next_dir)
        algorithm.append((direction-current_dir+1)%4-1)
        direction=int(next_dir)

    # for i in range(len(algorithm)):

    return algorithm


def matching(free_cubes, cubes_required, current=[], current_weight=0):
    global weights, best, order
    if free_cubes:
        for i in free_cubes:
            for i2 in cubes_required:
                free_cubes2=free_cubes.copy()
                cubes_required2=cubes_required.copy()
                free_cubes2.remove(i)
                cubes_required2.remove(i2)
                current2=current.copy()
                current2.append(list([i,i2]))
                kit = "".join([str(a) for a in list(sorted([i,i2]))])
                matching(free_cubes2, cubes_required2, current2, current_weight+weights.get(kit))
    else:
        if current_weight < best:
            best=current_weight
            order=current


def path_follow(path):
    for i in path:
        pid_reg(300, 1, 0, 0, 1)
        if i == 1:
            turn(1,300,100)
        elif i == -1:
            turn(1,-300,100)

def per_count(v,kp,ki,kd,ride=500):
    pers_list=[]
    error_i = 0
    last_error = 0
    n=0
    while (LS_center.reflection() < 75 or LS_right.reflection() < 75 or LS_left.reflection() < 75): #  or cube detected
        old_deg = left_motor.angle()
        while (LS_right.reflection() > 40 or LS_left.reflection() > 40) and (LS_center.reflection() < 75 or LS_right.reflection() < 75 or LS_left.reflection() < 75):
            
            error = LS_right.reflection() - LS_left.reflection()
            error_i = error + error_i
            error_d = error - last_error

            p = kp * error
            i = ki * error_i
            d = kd * error_d

            u = p + i + d
            
            last_error = error

            left_motor.run(v + u)
            right_motor.run(v - u)
            
            wait(10)

            if 30 < LS_center.reflection() < 40 and 30 < LS_right.reflection() < 40 and 30 < LS_left.reflection() < 40: #finish color between (30) and (40) - change
                return [-1, -1]
        if ride:
            left_motor.run(300)
            right_motor.run(300)
            wait(ride)
        n+=1
        pers_list.append(left_motor.angle() - old_deg)
        left_motor.stop()
        right_motor.stop()
    left_motor.stop()
    right_motor.stop()
    return [pers_list, n]

# def cross_arrangement():
#     global x, new_y
#     cubes_got = 0
#     for i in range(3):
#         if x != i:
#             # берёт куб, разворот, проезд к линии
#             ride_mm(300, 190)
#             ride_mm(300, 100*cubes_got)
#             close()
#             cubes_got += 1
#             turn(1, 300, 500)
#             pid_reg(300, 1, 0, 0, 1)


 
#             pid_reg_deg(300,1,0,0,mm_to_deg(i*210+190))
#             tank_turn(-90)
#             ride_mm(300, new_y*210+120)
#             open()
#             ride_mm(-300, new_y*210+120)
#             tank_turn(-90)
#             pid_reg(300, 1, 0, 0, 1)
#     pid_reg_deg(300,1,0,0,mm_to_deg(200))
#     tank_turn(90)
#     ride_mm(300, 200)






# def null_arrangement():
#     global x, new_y
#     cubes_got = 0
#     for i in range(3):
#         if x != i:
#             # берёт куб, разворот, проезд к линии
#             ride_mm(300, 190)
#             ride_mm(300, 100*cubes_got)
#             close()
#             cubes_got += 1
#             turn(1, -300, 500)
#             pid_reg(300, 1, 0, 0, 1)


 
#             pid_reg_deg(300,1,0,0,mm_to_deg(i*210+190))
#             tank_turn(90)
#             ride_mm(300, (2-new_y)*210+120)
#             open()
#             ride_mm(-300, (2-new_y)*210+120)
#             tank_turn(90)
#             pid_reg(300, 1, 0, 0, 1)
#     pid_reg_deg(300,1,0,0,mm_to_deg(200))
#     tank_turn(-90)
#     ride_mm(300, 200)
    


# конец функций
ghj=per_count(300, 5, 0, 0)
ev3.screen.print(ghj[0]) # test!
ev3.screen.print(ghj[1]) # test!
wait(10000)

while LS_cube.reflection() < 40:
    # ...
    ev3.screen.print(LS_left.reflection(), LS_center.reflection(), LS_right.reflection(), LS_cube.reflection())

# while LS_right.reflection()



# robot move programm
if per_count(300, 1, 0, 0)[0] !=-1:
    turn(2,300,100)
    pers_list, per_n=per_count(300, 1, 0, 0)
    if per_n == 2:
        turn(2,300,100)
        line_per(1)
        turn(1,-300,100)
        per_count()
        path_follow(pathfinding("0", "F"))
    elif per_n == 3:
        if pers_list[1] > pers_list[2]:
            path_follow(pathfinding("0", "F"))
        else:
            path_follow(pathfinding("1", "F"))
    elif per_n == 6:
        if pers_list[0] > pers_list[6]:
            path_follow(pathfinding("4", "F"))
        else:
            path_follow(pathfinding("3", "F"))
ride_mm(...)
turn(2,300,100)
code1, code2 = code_read()
code1=bin(int(code1[::-1],3))[2:]
code2=bin(int(code2[::-1],3))[2:]
code1="0"*(6-len(code1))+code1
code2="0"*(6-len(code2))+code2
code1=code1[::-1]
code2=code2[::-1]
cubes = []
for i in range(6):
    cubes.append(int(code1[i])-int(code2[i]))
# print(cubes)
weights = {"12":1, "13":4, "14":3, "15":5, "16":4, "23":4, "24":3, "25":5, "26":4, "34":1, "35":3, "36":3, "45":3, "46":3, "56":2}
# 1-5
free_cubes=[]
cubes_required=[]
for i in range(len(cubes)):
    if cubes[i]==1:
        free_cubes.append(i+1)
    elif cubes[i]==-1:
        cubes_required.append(i+1)
# print(free_cubes, cubes_required)

best=100
order=[]

matching(free_cubes, cubes_required)
# print(order, best)

current="F"
for i in order:
    path_follow(pathfinding(current, str(i[0])))
    path_follow(pathfinding(str(i[0]), str(i[1])))
    current=str(i[1])
path_follow(pathfinding(current, "F"))

# print(pathfinding("2", "3"))

# field
#       0
# F-----+--\  1  
#       |   \ |
#   /---+-\  \+-2
#   |   | |  |
# 6-+-\ \-+--/
#   | |   |
#   \-+---+---\
#     |   |   |
# 5---/   4   3
# krasota

# field with numbers (letters) of crosses
#       0
# F-----a--\  1  
#       |   \ |
#   /---b-\  \c-2
#   |   | |  |
# 6-d-\ \-e--/
#   | |   |
#   \-f---g---\
#     |   |   |
# 5---/   4   3

# field2
#       0
# F-----a--\  1
#       |   \ |
#   /---b-\  \c-2
#   |   | |   |
# 6-d-\ \-e---/
# 5-f-/   |
#   \-----g---\
#         |   |
#         4   3

# open()

# while not Button1.pressed():
#     # ev3.screen.print(US.distance())
#     ...

# # timer.reset()


# pid_reg(200, 0, 0, 0, 1, 600)
# tank_turn(-90)
# pid_reg(200, 0, 0, 0, 1)
# deg_enc = left_motor.angle()
# old = 2550
# pid_reg_deg(300,1,0,0,mm_to_deg(100))
# while US.distance() > 900 or (US.distance() - old) <= 4:
#     old = US.distance()
#     empty_pid_reg(300, 1, 0, 0, 10)
# wait(0)
# motors_stop()

# # ev3.screen.print(US.distance())

# if US.distance() < 250:
#     y1=0
#     new_y1=1
# elif US.distance() < 450:
#     y1=1
#     new_y1=2
# else:
#     y1=2
#     new_y1=1

# x1=floor((((left_motor.angle() - deg_enc) * 3.1415 * d / 360 - 20) / 210))

# ride_mm(300, 240)

# ev3.screen.print(y1)
# # ev3.screen.print(left_motor.angle()-deg_enc)
# ev3.screen.print(x1)

# char1 = (abs(x1-y1))%2 # 1 - O 0 - X

# ev3.screen.print(char1)
# old = US.distance()

# while (US.distance() > 800 or (US.distance() - old) <= 4) and (LS_right.reflection() > 40 or LS_left.reflection() > 40):
#     old = US.distance()
#     empty_pid_reg(300, 1, 0, 0, 10)
# wait(0)
# motors_stop()

# # ev3.screen.print(US.distance())
# if LS_right.reflection() > 40 or LS_left.reflection() > 40:
#     if US.distance() < 250:
#         y2=0
#     elif US.distance() < 450:
#         y2=1
#     else:
#         y2=2
#         # ride_mm(-300, 200)

#     x2=floor((((left_motor.angle() - deg_enc) * 3.1415 * d / 360 - 20) / 210))

#     ev3.screen.print(y2)
#     # ev3.screen.print(left_motor.angle()-deg_enc)
#     ev3.screen.print(x2)

#     char2 = (abs(x2-y2))%2 # 1 - O 0 - X

#     ev3.screen.print(char2)

#     pid_reg(300, 1, 0, 0, 1)
    
#     turn(2, -300, 300)

# else:
#     deg_enc = left_motor.angle()

#     pid_reg_deg(300,1,0,0,mm_to_deg(130))
#     tank_turn(-90)
#     while US.distance() > 900 or (US.distance() - old) <= 4:
#         old = US.distance()
#         empty_pid_reg(300, 1, 0, 0, 10)
#     ride_mm(300, 150)

#     old = US.distance()
#     while US.distance() > 900 or (US.distance() - old) <= 4:
#         old = US.distance()
#         empty_pid_reg(300, 1, 0, 0, 10)
#     wait(0)
#     motors_stop()

#     # ev3.screen.print(US.distance())

#     if US.distance() < 250:
#         x2=2
#     elif US.distance() < 450:
#         x2=1
#     else:
#         x2=0

#     y2=floor((((left_motor.angle() - deg_enc) * 3.1415 * d / 360 - 20) / 210))

#     ev3.screen.print(y2)
#     ev3.screen.print(x2)
#     char2 = (abs(x2-y2))%2 # 1 - O 0 - X

#     ev3.screen.print(char2)

    

#     tank_turn(160)
#     anti_turn(1, 300, 0)
    

#     pid_reg(300, 1, 0, 0, 1)
#     turn(2, 300, 300)
# both_char = int(char1 or char2)
# if abs(x1-x2)==0 or abs(y1-y2)==0:
#     both_char = 1
# ev3.screen.print(both_char)

# # ai
# if both_char:
#     if x1==x2:
#         x_target = x1
#         y_target = 3-y1-y2
#     elif y1==y2:
#         y_target = y1
#         x_target = 3-x1-x2
# else:
#     x_target = 3-x1-x2
#     y_target = 3-y1-y2

# ev3.screen.print([x_target, y_target])

# wait(10000)

# ride_mm(300, 150)
# tank_turn(-90)
# ride_mm(300, y*210+120)
# close()
# if y == 2:
#     ride_mm(-300, 210)
#     open()
#     ride_mm(-300, y*210-90)
# else:
#     ride_mm(300, 210)
#     open()
#     ride_mm(-300, y*210+330)


# if char:
#     tank_turn(-90)
#     pid_reg(200, 1, 0, 0, 1)
#     ride_mm(300, 200)
#     ride_mm(300, 50)
#     close()
#     turn(1, 300, 500)
#     if y==2:
#         pid_reg(300, 1, 0, 0, 2)
#         pid_reg_deg(300,1,0,0,mm_to_deg(170))
#         tank_turn(-90)
#         pid_reg(300, 1, 0, 0, 1)
#         tank_turn(-90)
#         pid_reg_deg(300,1,0,0,mm_to_deg(100))
#         pid_reg_deg(300,1,0,0,mm_to_deg((2-x)*210+120))
#         ride_mm(300, 150)
#         tank_turn(-90)
#         ride_mm(300, 150)
#         open()
#         ride_mm(-300, 150)
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 1)
#         null_arrangement()

#     else:
#         pid_reg(300, 1, 0, 0, 1)
#         pid_reg_deg(300,1,0,0,mm_to_deg(100))
#         pid_reg_deg(300,1,0,0,mm_to_deg(x*210+140))
#         tank_turn(-90)
#         ride_mm(300, y*210+120)
#         open()
#         ride_mm(-300, y*210+120)
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 1)
#         pid_reg_deg(300,1,0,0,mm_to_deg(170))
#         tank_turn(-90)
#         pid_reg(300, 1, 0, 0, 1)
#         tank_turn(-90)
#         pid_reg(300, 1, 0, 0, 2)
#         null_arrangement()


# else:
#     tank_turn(90)
#     pid_reg(300, 1, 0, 0, 1)
#     pid_reg_deg(300,1,0,0,mm_to_deg(170))
#     tank_turn(-90)
#     pid_reg(300, 1, 0, 0, 1)
#     tank_turn(-90)
#     pid_reg(300, 1, 0, 0, 2)

#     ride_mm(300, 200)
#     ride_mm(300, 50)
#     close()
#     turn(1, -300, 500)
#     if y==2:
#         pid_reg(300, 1, 0, 0, 1)
#         pid_reg_deg(300,1,0,0,mm_to_deg(100))
#         pid_reg_deg(300,1,0,0,mm_to_deg(x*210+140))
#         tank_turn(90)
#         ride_mm(300, 150)
#         open()
#         ride_mm(-300, 150)
#         tank_turn(-90)
#         pid_reg(300, 1, 0, 0, 1)
#         pid_reg_deg(300,1,0,0,mm_to_deg(170))
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 1)
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 2)
#         cross_arrangement()


#     else:
#         pid_reg(300, 1, 0, 0, 2)
#         pid_reg_deg(300,1,0,0,mm_to_deg(170))
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 1)
#         tank_turn(90)
#         pid_reg(300, 1, 0, 0, 1)
#         pid_reg_deg(300,1,0,0,mm_to_deg(100))
#         pid_reg_deg(300,1,0,0,mm_to_deg((2-x)*210+120))
#         tank_turn(90)
#         ride_mm(300, y*210+120)
#         open()
#         ride_mm(-300, y*210+120)
#         tank_turn(-90)
#         pid_reg(300, 1, 0, 0, 1)
#         cross_arrangement()













# for i in range(4):
#     pid_reg(300, 0.5, 0, 0, 1, 0)
#     ride_mm(-300, 30)
#     turn(1, 300, 400)
#     tank_turn(-10)
#     pid_reg(300, 0.5, 0, 0, 1, 0)


#     cube_number, cube_type = cube_search(6, 15, 60) # 1 - твёрдый, 0 - мягкий, 2 - высокий


#     ev3.screen.print(cube_number)
#     ev3.screen.print(cube_type)

#     if cube_number <= 1:
#         tank_turn(180)
#         ride_mm(-300, 50)
#     else:
#         tank_turn(100)
#         turn(1, 300, 300)
#         tank_turn(-10)

#     if cube_number <= 2:
#         pid_reg(300, 1, 0, 0, 1)
#     else:
#         pid_reg(300, 1, 0, 0, 2)

#     turn(1, 300, 700)
#     tank_turn(-10)
#     pid_reg(300, 1, 0, 0, 1)
#     ride_mm(300, 30)

#     pid_reg_deg(300, 0.5, 0, 0, mm_to_deg(55))

#     pid_reg_deg(300, 1, 0, 0, mm_to_deg(295)*(cube_number+1))



#     tank_turn(-100 * int(2*(bool(cube_type)-0.5)))

#     ride_mm(300, (150*((cube_type == 2))+50))

#     open()

#     ride_mm(-300, (150*((cube_type == 2))+50))

#     tank_turn(-90 * int(2*(bool(cube_type)-0.5)))
#     turn(-1*int(2*(bool(cube_type)-0.5)), 300, 50)


#     pid_reg(300, 1, 0, 0, 2)

#     pid_reg_deg(300, 0.5, 0, 0, mm_to_deg(45))

#     turn(2, 300, 600)
#     tank_turn(-10)


# wait(10000)







# алгоритм
# поворот
# проезд до перекрёстка
# проезд
# проезд, поиск зоны (cube_search)
# возврат
# поворот
# проезд до зоны
# проезд по градусам
# поворотъ
# проезд
# поворот
# установка
# возврат на линию
# разворот
# езда к старту
# разворот
# ...
# поворот налево
# проезд до перекрёстка
# проезд
# проезд, поиск зоны (cube_search)
# возврат
# поворот
# проезд до зоны
# проезд по градусам
# поворотъ
# проезд
# поворот
# установка
# возврат на линию
# разворот
# езда к старту
# разворот

# определение:
# едет пока не увидит расстояние отличное от расстояния до пола
# если больше, то мягкий
# если меньше, то жёсткий








# mark_x = [0, 0, 0, 0, 1, 1, 1, 1]
# mark_y = [0, 1, 2, 3, 0, 1, 2, 3]
# mark_color = [1, 0, 3, 2, 2, 1, 0, 3] # 0 -красный, 1 - желтый, 2 - зелёный, 3 - синий.
# x = 0
# y = 0


# to_first()

# # pid_reg(850, 13, 0.1, 0, 2)
# # pid_reg(500, 5, 0, 0, 2)


# # ev3.screen.print(timer.time())

# wait(5000)

# # ev3.screen.print(zone3(3, 140))







# Алгоритм:
# Создать список цветов меток и их координат (0 - слева, 1 - справа. 0 - 3 координата по удалению от старта (можно 3 списка, можно словарь, координаты: цвет))
# Создать переменные для текущих координат изначально 0, 0, едем в 0, 0.
# Доехать до куба (0, 0) и прочитать.
# Цикл пока список не пуст:
#   Если куб стоит на своём цвете, то оставить и запомнить (удалить), иначе - ехать к ближайшей метке (расстояние = модуль(текущая позиция - позиция куба (x не важен)).
#   После приезда к ближайшей метке замена куба и удаление из списка, перед этим задать текущую позицию.
# Считывание куба.
# Возвращение на финиш (можно ближайший)

# Функции:
# to_first() к первому кубу.
# replace() замена куба.
# cube_read() считывание цвета.
# get(x, y) езда к координате.
# data_change(x, y, color) меняет данные (удаляет из списков)
# closest_search(x, y, color) ищет ближайшую позицию, обновляет целевую позицию.

