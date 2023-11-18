import re

FileName = r"C:\Users\Ericw\Desktop\TopScience2.txt"

x=[]
y=[]

with open(FileName,"r", encoding='utf-8-sig') as f:
    lines = f.read()
    lines2 = lines.splitlines()

    for line in lines2:
        result = (re.search(r"X([\S]+) Y([\S]+)", line)).groups()

        x_temp = result[0]
        y_temp = (result[1])

        x.append(float(x_temp))
        y.append(float(y_temp))

print(*zip(x,y))

