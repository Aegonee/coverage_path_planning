import csv
import numpy as np

data=[]
slicepara=720               #总数据分成a部分
para=6                      #a中每多少个元素保留一个数据
reserved=256                #n*m*l中的n的取值
path='tran_data-x.csv'      #csv数据路径，假如与Py文件放在同一文件夹内则只需填写文件名

file=open(path,'r')
reader=csv.reader(file)
print(type(reader))
for i in reader:
    data.append(i)
file.close

data.pop(0)
for i in range(len(data)):
    data[i].pop(len(data[i])-1)
    data[i].pop(0)
    for j in range(len(data[i])):
        data[i][j]=float(data[i][j])


slice1=len(data)//slicepara
temp_result=[]
    
for i in range(slice1):
    temp=[]
    for j in range(i*720,(i+1)*720):
        if (j+1)%para==0:
            temp.append(data[j])
    temp_result.append(temp)

while len(temp_result)>reserved:
    temp_result.pop(len(temp_result)-1)

tensor=np.array(temp_result)

print(tensor)