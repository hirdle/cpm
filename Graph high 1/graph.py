#!/usr/bin/env pybricks-micropython
from math import floor, ceil
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

edges_list = [["А1","Б1",24],["А1","А2",31],["Б1","В1",41],["Б1","Б2",43],["В1","Г1",49],["В1","В2",23],["Г1","Д1",45],["Г1","Г2",10],["Д1","Е1",30],["Д1","Д2",23],["Е1","Ж1",36],["Е1","Е2",40],["Ж1","Ж2",24],["А2","Б2",18],["А2","А3",27],["Б2","В2",49],["Б2","Б3",33],["В2","Г2",20],["В2","В3",24],["Г2","Д2",10],["Г2","Г3",43],["Д2","Е2",25],["Д2","Д3",22],["Е2","Ж2",44],["Е2","Е3",12],["Ж2","Ж3",31],["А3","Б3",21],["А3","А4",40],["Б3","В3",22],["Б3","Б4",15],["В3","Г3",15],["В3","В4",44],["Г3","Д3",47],["Г3","Г4",42],["Д3","Е3",34],["Д3","Д4",41],["Е3","Ж3",33],["Е3","Е4",49],["Ж3","Ж4",46],["А4","Б4",46],["А4","А5",17],["Б4","В4",24],["Б4","Б5",33],["В4","Г4",20],["В4","В5",49],["Г4","Д4",15],["Г4","Г5",19],["Д4","Е4",12],["Д4","Д5",19],["Е4","Ж4",14],["Е4","Е5",50],["Ж4","Ж5",37],["А5","Б5",23],["Б5","В5",45],["В5","Г5",17],["Г5","Д5",26],["Д5","Е5",50],["Е5","Ж5",46]]
graph = {}

timer = StopWatch()

left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

LS_right = ColorSensor(Port.S2)
LS_left = ColorSensor(Port.S3)

Button1 = TouchSensor(Port.S1)

S = 0
d = 62.4
deg_enc = left_motor.angle()

number = 0

error_i = 0
last_error = 0

filtered = [0, 0, 0]

robot = DriveBase(left_motor, right_motor, wheel_diameter=64, axle_track=104)

# функции из мм в градусы и наоборот
def mm_to_deg(mm):
    global d
    return mm * 360 / d / 3.1415


def deg_to_mm(deg):
    global d
    return deg / 360 * d * 3.1415


# стоп моторы
def motors_stop():
    left_motor.stop()
    right_motor.stop()


# ПИД регулятор по перекресткам
def pid_reg(v,kp,ki,kd,n,ride=35):
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
            ride_mm(300, ride)
            # left_motor.run(300)
            # right_motor.run(300)
            # wait(ride)
        left_motor.stop()
        right_motor.stop()


# ПИД регулятор по градусам
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


# непонятно
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


# тоже непонятно
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


# фильтры
def filter(data, n):
    global filtered
    filtered.append(data)
    if len(filtered) >= 2*n:
        filtered = filtered[-1*n:]
        # ev3.screen.print(filtered[:n])
    return sorted(filtered[:n])[n//2]


# ввод чиселок
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


# определение зоны
def zone(n, x_0):
    global deg_enc, d
    pi_reg(300, 0.3, 0, 0, 1)
    S = (left_motor.angle() - deg_enc) * 3.1415 * d / 360
    return floor(3 - ((S - x_0) / 300))


# определение зоны модифицированное
def zone2(n, y_0):
    # pi_reg(300, 0.3, 0, 0, 1)
    y_sum = 210
    S = US.distance()
    return ceil((S - y_0) / (y_sum / n))


# определение зоны новое
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


# танковый поворот
def tank_turn(degrees):
    global d
    deg = left_motor.angle()
    while abs(left_motor.angle() - deg) < abs(degrees) * 180 / d:
        left_motor.run(300 * degrees // abs(degrees))
        right_motor.run(-300 * degrees // abs(degrees))
    left_motor.stop()
    right_motor.stop()


# поворот танковый по датчикам
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


# анти-поворот танковый по датчикам
def anti_turn(n, v, t, back=0):
    for _ in range(n):
        left_motor.run(v)
        right_motor.run(-v)
        wait(t)
        while (LS_left.reflection() * (v > 0)) + (LS_right.reflection() * (v < 0)) > 20:
            left_motor.run(v)
            right_motor.run(-v)
        left_motor.run(v)
        right_motor.run(-v)
        wait(back)
        motors_stop()


# выравнивание по линии
def line_align(v):
    while (LS_left.reflection() > 40 or LS_right.reflection() > 40):
        if LS_left.reflection() > 40 and LS_right.reflection() > 40:
            left_motor.run(-v)
            right_motor.run(-v)
        elif LS_right.reflection() < 40:
            left_motor.run(v)
            right_motor.run(-v)
        elif LS_left.reflection() < 40:
            left_motor.run(-v)
            right_motor.run(v)
    motors_stop()


# финиширование робота
def finish():
    pi_reg(300, 0.3, 0.003, 0, 1)    
    robot.straight(150)
    tank_turn(90 * (number - 2))


# проезд в мм
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


# открыть захват
def open():
    mid_motor.run_until_stalled(300, duty_limit=50)

# закрыть захват
def close():
    mid_motor.run_until_stalled(-300, duty_limit=50)
    mid_motor.hold()


def cube_search(n, x_0, l):
    global deg_enc, d
    deg_enc = left_motor.angle()
    for _ in range(10):
        value = filter(US.distance(), 5)
    while 90 < value < 125:
        value = filter(US.distance(), 5)
        empty_pid_reg(200, 0.3, 0, 0)
    cube_type = value
    motors_stop()
    S = (left_motor.angle() - deg_enc) * 3.1415 * d / 360
    # ev3.screen.print(cube_type)
    # ev3.screen.print(floor(((S - x_0) / l)))
    # ev3.screen.print(S)
    close()
    if cube_type < 75:
        return floor(((S - x_0) / l)), 2
    return floor(((S - x_0) / l)), int(cube_type <= 100)


# строим дерево путей
def tree_path(current: int, length: int, min_:int, path: list, end_: int, best_path: list, old: int = 99):
    global graph
    if current == end_:
        return [length, path]
    if length < min_:
        if current%7 != 6:
            if current+1 != old:
                path2=path.copy()
                path2.append(current+1)
                result = tree_path(current+1, length+graph.get(str(current)+" "+str(current+1)), min_, path2, end_, best_path, current)
                if result:
                    if result[0] < min_:
                        min_=result[0]
                        best_path=result[1]
        if current//7 != 0:
            if current-7 != old:
                path2=path.copy()
                path2.append(current-7)
                result = tree_path(current-7, length+graph.get(str(current-7)+" "+str(current)), min_, path2, end_, best_path, current)
                if result:
                    if result[0] < min_:
                        min_=result[0]
                        best_path=result[1]
        if current//7 != 4:
            if current+7 != old:
                path2=path.copy()
                path2.append(current+7)
                result = tree_path(current+7, length+graph.get(str(current)+" "+str(current+7)), min_, path2, end_, best_path, current)
                if result:
                    if result[0] < min_:
                        min_=result[0]
                        best_path=result[1]
        if current%7 != 0:
            if current-1 != old:
                path2=path.copy()
                path2.append(current-1)
                result = tree_path(current-1, length+graph.get(str(current-1)+" "+str(current)), min_, path2, end_, best_path, current)
                if result:
                    if result[0] < min_:
                        min_=result[0]
                        best_path=result[1]
        return [min_, best_path]


# поиск пути по коду
def pathfinding(code2):
    global edges_list, graph
    # 0  1  2  3  4  5  6
    # 7  8  9  10 11 12 13
    # 14 15 16 17 18 19 20
    # 21 22 23 24 25 26 27
    # 28 29 30 31 32 33 34

    for i in edges_list:
        first_point = ord(i[0][0])-1040+int(i[0][1])*7-7
        second_point = ord(i[1][0])-1040+int(i[1][1])*7-7
        graph.update({str(min(first_point, second_point))+" "+str(max(first_point, second_point)): i[2]})

    code=""
    for i in code2:
        code = i + code

    end_ = str(int(code, 2))
    end_ = int(end_[0])+(int(end_[1])-1)*7-1

    # end_ = int(input())

    length, path = tree_path(14, 0, 50*(abs(end_//7-2)+end_%7), [], end_, [])
    # print(length, path)

    path.insert(0, 14)

    # print(path)
    algorithm = [] # 0 - forward, 1 - right, 2 - left
    direction = 0 # 0 - forward, 1 - right, 2 - left, 3 - backward
    for i in range(len(path)-1):
        if path[i + 1] - path[i] == 1:
            if direction == 1:
                algorithm.append(2)
            if direction == 2:
                algorithm.append(1)
            if direction == 3:
                algorithm.append(1)
                algorithm.append(1)
            direction = 0

        if path[i + 1] - path[i] == -1:
            if direction == 0:
                algorithm.append(1)
                algorithm.append(1)
            if direction == 1:
                algorithm.append(1)
            if direction == 2:
                algorithm.append(2)
            direction = 3

        if path[i + 1] - path[i] == 7:
            if direction == 0:
                algorithm.append(1)
            if direction == 2:
                algorithm.append(1)
                algorithm.append(1)
            if direction == 3:
                algorithm.append(2)
            direction = 1

        if path[i + 1] - path[i] == -7:
            if direction == 0:
                algorithm.append(2)
            if direction == 1:
                algorithm.append(1)
                algorithm.append(1)
            if direction == 3:
                algorithm.append(1)
            direction = 2
        algorithm.append(0)
    return algorithm

# считывание кода
def code_read():
    global d
    data = ""
    deg = left_motor.angle()
    while LS_right.reflection() < 45:
        left_motor.run(200)
        right_motor.run(200)
    ev3.speaker.beep(100)
    wait(30)
    left_motor.stop()
    right_motor.stop()
    deg1=left_motor.angle()
    while LS_right.reflection() > 45:
        left_motor.run(200)
        right_motor.run(200)
    ev3.speaker.beep(100)
    wait(30)
    left_motor.stop()
    right_motor.stop()
    deg2=left_motor.angle()
    deg_target=(deg2-deg1)//2+1
    ride_mm(200, deg_target)
    for i in range(7):
        ev3.speaker.beep(100)
        data+=str(int(LS_right.reflection() < 45))
        ride_mm(200, deg_target)
    ride_mm(200, 100)
    return data


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

# while not Button1.pressed():
#     ...
# for i in range(16):
#     pid_reg(450, 1.5, 0, 0, 1)
#     anti_turn(1, -350, 250, 125)

# wait(1000)

# ожидаем нажатия кнопки (тач сенсора)
while not Button1.pressed():
    ...

# проезд до кода
ride_mm(400, 50)
wait(100)
line_align(200)
ride_mm(400, 50)
wait(100)

pid_reg(400, 1.75, 0, 0, 1, 0)

# считываем код
data=code_read()
# выводим его
ev3.screen.print(data)
# ищем путь для проезда
algorithm = pathfinding(data)
# небольшой поворот для выравнивания
tank_turn(5)

# проезд до основной (стартовой) точки
pid_reg(450, 2, 0, 0, 1, 40)
# pid_reg(300, 1, 0, 0, 1, 600)

# проезд по алгоритму ( 0 - проезд прямо, 1 - поворот направо, 2 - поворот налево )
for i in algorithm:
    if i==0:
        pid_reg(450, 1.75, 0, 0, 1)
    if i==1:
        anti_turn(1, 350, 250, 125)
        pid_reg_deg(450, 1.75, 0, 0, 100)
    if i==2:
        anti_turn(1, -350, 250, 125)
        pid_reg_deg(450, 1.75, 0, 0, 100)


# здесь разворот робота на 180 градусов
wait(3000)
tank_turn(180)
# anti_turn(1, 350, 250, 125)
# turn(1, 300, 0, 0)

print(algorithm)

# проезд по обратному пути
for i in range(len(algorithm)):
    if algorithm[len(algorithm)-i-1]==0:
        ride_mm(300, 20)
        pid_reg(450, 2, 0, 0, 1)
    if algorithm[len(algorithm)-i-1]==2:
        anti_turn(1, 350, 250, 125)
        pid_reg_deg(450, 1.75, 0, 0, 100)
    if algorithm[len(algorithm)-i-1]==1:
        anti_turn(1, -350, 250, 125)
        pid_reg_deg(450, 1.75, 0, 0, 100)
    print(algorithm[len(algorithm)-i-1])


# едем обратно на финиш
line_align(200)
pid_reg_deg(450, 1.75, 0, 0, 100)
pid_reg(450, 2, 0, 0, 1, 0)
# pid_reg(300, 1, 0, 0, 1, 0)
ride_mm(300, 300)
pid_reg_deg(450, 1.5, 0, 0, 200)
ride_mm(300, 130)






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

