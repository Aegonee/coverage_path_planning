import re
import numpy as np

a='12'
b='12.2,13.2,13.3,14'

print(type(b[0]))
print(float(b[1]))
print(float(-3.21))
print(re.findall("-?\d+\.?\d*e?-?\d+?",b))
print(re.match('[01][01][01][01][01][01]','100102'))
print(re.findall('\D+'))

c=[1,2,[2,3,[4,5,[6,7]]]]
print((np.array(c)).shape)