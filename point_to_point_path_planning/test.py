from multiprocessing import Process, cpu_count

def my_function(x):
    # Your code here
    pass

if __name__ == '__main__':
    processes = []
    n_cpu = cpu_count()
    for i in range(n_cpu):
        p = Process(target=my_function, args=(i,))
        p.start()
        processes.append(p)
    for p in processes:
        p.join()
    pass