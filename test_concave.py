from MultiAgent import AreaNode, AreaTree, concavearea, zigzagarea, plot_single_cpp
import matplotlib.pyplot as plt
# a = [(116.5201597, 28.21727518), (116.6339156, 28.17279978), (116.5871566, 28.2459996), (116.6542414, 28.17871836), (116.6762776, 28.24368918), (116.6768312, 28.11542968), (116.7379872, 28.11065993), (116.61967, 28.08649246), (116.6202553, 28.16272491)]
# a = [(113.18, 34.33), (113.10, 34.31), (113.18, 34.38), (113.22, 34.32), (113.34, 34.37), (113.24, 34.31), (113.23, 34.24), (113.19, 34.31), (113.16, 34.23), (113.08, 34.26)]
a = [(116.49, 28.38), (116.55, 28.63), (116.58, 28.47), (116.52, 28.43), (116.66, 28.35), (116.75, 28.37), (116.66, 28.33), (116.44, 28.31), (116.35, 28.39), (116.38, 28.58)]
a.reverse()
# a = [(0,0), (5,0), (4,1), (5,3), (2,2), (1,4), (3,8), (-1,2)]
# a = [(0,0),(5,0),(8,-3),(6,5),(3,2),(0,5)]
# a = [(1,1),(6,0),(4,2),(5,5),(0,6),(2,4)] #???
# a = [(0,0),(5,0),(3,3),(4,7),(0,6),(3,6)] #???
# a = [(2,0),(6,2),(9,-1),(7,7),(3,5),(0,8)] #clockwise correct but conter- not correct
# a = [(2,0),(0,8),(3,5),(7,7),(9,-1),(6,2)]
# a = [(0,0),(2,0),(2,-5),(4,2),(6,-5),(8,0),(4,4)] #solved
# a = [(0,0),(2,0),(2,-5),(4,2),(6,-5),(6,0),(8,0),(4,4)] #solved
# a= [(0,0), (4,0), (4,4), (3,4), (2,2), (1,4), (0,4)]
# a= [(0,0), (3,2), (5,0), (4,6), (1,4), (-1,6)]
b = [(2,2), (1,4), (3,8), (-1,2), (0,0), (3,0), (5,0), (4,1), (5,3), (9,9)] 
area = [(0,0), (6,0), (4,2), (8,3), (6,6), (1,6), (2,2)]

print(set(b)-set(a))

a_zigzag = concavearea(a)
print("area a is {0}".format(a_zigzag.is_concave))


# c = MultiAgent.concavearea(area)
# print(c.GrahamHull())
# print(c.check_concave())
# print(c.concave_index)
# print(c.concave_point)


d = AreaNode(a)
d_tree = AreaTree(d)

d_tree.convex_tree_generation()
print()
print("asdasdasd")
asd = d_tree.find_leaves(d_tree.root)
for i in asd:
    print(i.area)
    print('???')
print()

fig = plt.figure()
ax = fig.add_subplot(111)

x=[]
y=[]
for i in a:
    x.append(i[0])
    y.append(i[1])
x.append(a[0][0])
y.append(a[0][1])
plt.plot(x,y)
# '''
for i in range(len(asd)):
    pass
    # plot_single_cpp(asd[i].area, 0.02, 0.02, 0.01, 0.005, (113.2, 34.26), str(i)+'.txt', './img/'+str(i)+'.png')

    # x=[]
    # y=[]
    # temp = asd[i]
    # for j in range(len(temp.area)):
    #     x.append(temp.area[j][0])
    #     y.append(temp.area[j][1])
    # x.append(temp.area[0][0])
    # y.append(temp.area[0][1])
    # plt.plot(x, y)
# '''
ax.grid(True)
plt.legend(loc='upper right', frameon=True)

plt.show()

print(bin(3))

