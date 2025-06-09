# with graph work

# field
#       0
# F-----+--\  1
#       |   \ |
#   /---+-\  \+-2
#   |   | |   |
# 6-+-\ \-+---/
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
#   |   | |   |
# 6-d-\ \-e---/
#   | |   |
#   \-f---g---\
#     |   |   |
# 5---/   4   3


# graph in format ({cross_name: [up, left, down, right]})
graph = {"a": ["0", "F", "b", "c"],
         "b": ["a", "d", "e", "e"],
         "c": ["1", "a", "e", "2"],
         "d": ["b", "6", "f", "f"],
         "e": ["b", "b", "g", "c"],
         "f": ["d", "d", "5", "g"],
         "g": ["e", "f", "4", "3"]}
graph_dir = {"a": ["00", "F1", "b2", "c3"],
         "b": ["a0", "d2", "e3", "e2"],
         "c": ["10", "a1", "e1", "23"],
         "d": ["b3", "61", "f3", "f2"],
         "e": ["b1", "b0", "g2", "c0"],
         "f": ["d1", "d0", "51", "g3"],
         "g": ["e0", "f1", "42", "32"]}
# direction at end 0-up, 1-left, 2-down, 3-right

directions = {"F": "3", "0": "2", "1": "2", "2": "1", "3": "1", "4": "0", "5": "0", "6": "3"}
from_punkt = {"F": "a", "0": "a", "1": "c", "2": "c", "3": "g", "4": "g", "5": "f", "6": "d"}
straight_paths = ["0abec1", "Fac2", "3gfdbeg4", "5fd6"]

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
        # forward
        if i == 1:
#             right
           ...
        elif i == -1:
#             left
           ...



# robot move programm
# if per_count() !=-1:
#   tank_turn(...,2,...)
#   per_n, pers_list=per_count()
#   match per_n:
#       case 2:
#           if pers_list[0] > pers_list[2]:
#               path_follow(pathfinding("6", "F"))
#           else:
#               path_follow(pathfinding("5", "F"))
#       case 3:
#           if pers_list[1] > pers_list[2]:
#               path_follow(pathfinding("0", "F"))
#           else:
#               path_follow(pathfinding("1", "F"))
#       case 6:
#           if pers_list[0] > pers_list[6]:
#               path_follow(pathfinding("4", "F"))
#           else:
#               path_follow(pathfinding("3", "F"))
#       case -1:...
# drive_mm(...)
# tank_turn(2)
# code1, code2 = code_read()
# code1=bin(int(code1[::-1],3))[2:]
# code2=bin(int(code2[::-1],3))[2:]
# code1="0"*(6-len(code1))+code1
# code2="0"*(6-len(code2))+code2
# code1=code1[::-1]
# code2=code2[::-1]
# cubes = []
# for i in range(6):
#     cubes.append(int(code1[i])-int(code2[i]))
# # print(cubes)
# weights = {"12":1, "13":4, "14":3, "15":5, "16":4, "23":4, "24":3, "25":5, "26":4, "34":1, "35":3, "36":3, "45":3, "46":3, "56":2}
# # 1-5
# free_cubes=[]
# cubes_required=[]
# for i in range(len(cubes)):
#     if cubes[i]==1:
#         free_cubes.append(i+1)
#     elif cubes[i]==-1:
#         cubes_required.append(i+1)
# # print(free_cubes, cubes_required)
#
# best=100
# order=[]
#
# matching(free_cubes, cubes_required)
# # print(order, best)
#
# current="F"
# for i in order:
#     path_follow(pathfinding(current, str(i[0])))
#     path_follow(pathfinding(str(i[0]), str(i[1])))
#     current=str(i[1])
# path_follow(pathfinding(current, "F"))


start_ = input("start:\n")
end = input("end:\n")

# start_="4"
# end="F"
print(pathfinding(start_, end))

code1=bin(int(input("code1:\n")[::-1],3))[2:]
code2=bin(int(input("code2:\n")[::-1],3))[2:]
# code1=bin(int("0222",3))[2:]
# code2=bin(int("0111",3))[2:]
code1="0"*(6-len(code1))+code1
code2="0"*(6-len(code2))+code2
code1=code1[::-1]
code2=code2[::-1]
# print(code1)
# print(code2)


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
print(free_cubes, cubes_required)

best=100
order=[]

matching(free_cubes, cubes_required)
print(order, best)



current="F"
for i in order:
    print(pathfinding(current, str(i[0])))
    print(pathfinding(str(i[0]), str(i[1])))
    current=str(i[1])
print(pathfinding(current, "F"))


# print(pathfinding("2", "3"))


# 1. ехать по линии, пока один из датчиков черный

# 2. отъехать развернуться на перекрёстке
# (или просто разворот, без отъезда)

# 3. ехать по линии, пока один из датчиков черный, считая
# перекрёстки (создать функцию per_count) она будет
# возвращать количество перекрёстков и список длин
# участков по энкодеру

# важная заметка для per_count!!! сделать
# определение финиша - по окончании, когда все датчики
# на белом, если при небольшом отъезде,
# все датчики покажут чёрный, то мы находимся на финише,
# в таком случае вернуть -1, если при применении этой
# функции на 1 пункте будет получен -1, то
# перейти к пункту 4


# 4. теперь, если количество перекрёстков (per_n) == 4
# 	мы находимся на пунктах 1 или 0
# 	(том, что ближе к финишу)
# 	чтобы определить точно, нужно сравнить участок
# 	1 и 2 по индексу в списке, полученном из
# 	per_count, если 1 > 2, то мы на пункте 0,
# 	иначе на 1

# если per_n == 3:
# 	мы находимся на пунктах 5 или 6
# 	чтобы определить точно, нужно сравнить участок
# 	0 и 2 (-1) по индексу в списке, полученном из
# 	per_count, если 0 > 2, то мы на пункте 6,
# 	иначе на 5

# если per_n == 7:
# 	мы находимся на пунктах 3 или 4
# 	чтобы определить точно, нужно сравнить участок
# 	0 и 6 (-1) по индексу в списке, полученном из
# 	per_count, если 0 > 6, то мы на пункте 4,
# 	иначе на 3

# при per_n 3 4 и 7, алгоритм проезда к финишу 1,
# можно прописать все случаи, можно помучаться
# с графом

# если per_n == -1: проехать, развернуться

# 5. считать код
# 6. распределить с каких куда отвозить
# 7. отвезти, можно прописать все случаи, но если я из
# будущего помучался с графами, то не нужно