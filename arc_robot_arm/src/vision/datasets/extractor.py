import os

print("Enter folder name to extract file name types from: ")
folder_name = input()

print("Enter file to append extraction: ")
file_name = input()

mypath = os.path.dirname(os.path.abspath(__file__)) + "/" + folder_name
data = os.listdir(mypath)

data = set(map(lambda x: "_".join(x.split("_")[:-1]),data))
data = list(filter(lambda x: x[0] != '.' ,data))

with open(file_name, "a") as f:
    for name in data:
        f.write(" - ![%(name)s](%(dir_n)s/%(name)s_0.jpg)\n" % {"name" : name, "dir_n" : folder_name})
