import random
import math
import matplotlib.pyplot as plt
from multiprocessing import Pool
import time


def distance(point1, point2):
    """计算两点之间的距离"""
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)



def simulated_annealing(points, initial_temperature, cooling_rate):
    """模拟退火算法求解最短巡航路径"""
    
    # 随机生成初始路径
    current_path = random.sample(points, len(points))
    
    # 计算当前路径的总长度
    current_distance = sum(distance(current_path[i], current_path[i+1]) for i in range(len(current_path)-1))
    
    # 初始化最优路径和最优距离
    best_path = current_path.copy()
    best_distance = current_distance
    
    # 初始化当前温度
    temperature = initial_temperature
    
    # 迭代搜索
    while temperature > 0.1:
        # 随机选择两个不同的位置交换
        index1, index2 = random.sample(range(len(points)), 2)
        
        # 生成新的路径
        new_path = current_path.copy()
        new_path[index1], new_path[index2] = new_path[index2], new_path[index1]
        
        # 计算新路径的总长度
        new_distance = sum(distance(new_path[i], new_path[i+1]) for i in range(len(new_path)-1))
        
        # 计算能量差
        energy_difference = new_distance - current_distance
        
        # 判断是否接受新路径
        if energy_difference < 0 or random.random() < math.exp(-energy_difference / temperature):
            current_path = new_path
            current_distance = new_distance
        
        # 更新最优路径和最优距离
        if current_distance < best_distance:
            best_path = current_path.copy()
            best_distance = current_distance
        
        # 降低温度
        temperature *= cooling_rate
    
    return best_path, best_distance



def calculate_path_length(path):
    """计算路径的总长度"""
    total_length = sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
    return total_length

def initialize_population(points, population_size):
    """初始化种群"""
    population = []
    for _ in range(population_size):
        path = points.copy()
        random.shuffle(path)
        population.append(path)
    return population

def crossover(parent1, parent2):
    """交叉操作"""
    # 选择一个随机子序列
    start = random.randint(0, len(parent1) - 1)
    end = random.randint(start + 1, len(parent1))
    
    # 从父代中获取子序列
    child = parent1[start:end]
    
    # 填充剩余位置
    for gene in parent2:
        if gene not in child:
            child.append(gene)
    
    return child

def mutate(path, mutation_rate):
    """变异操作"""
    if random.random() < mutation_rate:
        # 选择两个随机位置进行交换
        index1 = random.randint(0, len(path) - 1)
        index2 = random.randint(0, len(path) - 1)
        path[index1], path[index2] = path[index2], path[index1]
    return path

def genetic_algorithm(points, population_size, generations, crossover_rate, mutation_rate):
    """遗传算法求解最短路径"""
    
    # 初始化种群
    population = initialize_population(points, population_size)
    
    # 迭代进化
    for _ in range(generations):
        # 计算适应度，即路径长度
        fitness_scores = [1 / calculate_path_length(path) for path in population]
        
        # 选择父代
        parents = random.choices(population, weights=fitness_scores, k=population_size)
        
        # 创建新一代种群
        new_population = []
        for _ in range(population_size // 2):
            parent1 = random.choice(parents)
            parent2 = random.choice(parents)
            
            # 交叉操作
            child1 = crossover(parent1, parent2)
            child2 = crossover(parent2, parent1)
            
            # 变异操作
            child1 = mutate(child1, mutation_rate)
            child2 = mutate(child2, mutation_rate)
            
            new_population.extend([child1, child2])
        
        population = new_population
    
    # 找到最优路径
    best_path = min(population, key=calculate_path_length)
    best_distance = calculate_path_length(best_path)
    
    return best_path, best_distance

# 创建点集
points = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(100)]

x = []
y = []
for i in range(len(points)):
    t_x, t_y = points[i]
    x.append(t_x)
    y.append(t_y)      

# 设置模拟退火算法参数
initial_temperature = 100
cooling_rate = 0.99

# 调用模拟退火算法求解最短巡航路径
start_time_SA = time.time()
best_path, best_distance = simulated_annealing(points, initial_temperature, cooling_rate)
end_time_SA = time.time()
print("completed!")

start_time_GA = time.time()
test_best_path, test_best_distance = genetic_algorithm(points, 100, 500, 0.8, 0.1)
end_time_GA = time.time()

path_x = []
path_y = []

test_path_x = []
test_path_y = []

for i in range(len(best_path)):
    t_x, t_y = best_path[i]
    path_x.append(t_x)
    path_y.append(t_y)

for i in range(len(test_best_path)):
    t_x, t_y = test_best_path[i]
    test_path_x.append(t_x)
    test_path_y.append(t_y)

# 输出结果
#print("模拟退火算法的最短巡航路径:", best_path)
print("模拟退火算法的最短路径长度:", best_distance)
print("模拟退火算法运行时间：", start_time_SA - end_time_SA)
#print("遗传算法检验的最短巡航路径:", test_best_path)
print("遗传算法检验的最短路径长度:", test_best_distance)
print("遗传算法运行时间：", start_time_GA - end_time_GA)

fig = plt.figure()
ax = fig.add_subplot(121)
ax.set(title='Simulated Annealing Algorithm', ylabel='y', xlabel='x')
ax2 = fig.add_subplot(122)
ax2.set(title='Genetic Algorithm', ylabel='y', xlabel='x')
ax.plot(x, y, "o")
ax2.plot(x, y, "o")
ax.plot(path_x, path_y, "--")
ax2.plot(test_path_x, test_path_y, "--")
ax.plot(path_x[0], path_y[0], "o", label = "Start Point")
ax2.plot(test_path_x[0], test_path_y[0], "o", label = "Start Point")
ax.plot(path_x[len(path_x)-1], path_y[len(path_y)-1], "o", label = "End Point")
ax2.plot(test_path_x[len(path_x)-1], test_path_y[len(path_y)-1], "o", label = "End Point")
ax.grid(True)
ax2.grid(True)
ax.legend(loc='upper right',frameon=True)
ax2.legend(loc='upper right',frameon=True)
plt.show()

f = open("points.txt","w")
for i in range(len(points)):
    f.write(str(points[i][0]))
    f.write("\t")
    f.write(str(points[i][1]))
    f.write("\n")
f.close()

f = open("Result_SA.txt","w")
for i in range(len(best_path)):
    f.write(str(best_path[i][0]))
    f.write("\t")
    f.write(str(best_path[i][1]))
    f.write("\n")
f.close()

f = open("Result_GA.txt","w")
for i in range(len(test_best_path)):
    f.write(str(test_best_path[i][0]))
    f.write("\t")
    f.write(str(test_best_path[i][1]))
    f.write("\n")
f.close()