import os

print("Enter folder name to extract file name types from: ")
folder_name = input()

mypath = os.path.dirname(os.path.abspath(__file__)) + "/" + folder_name
data = os.listdir(mypath)

data = set(map(lambda x: "_".join(x.split("_")[:-1]),data))
data = list(filter(lambda x: x[0] != '.' ,data))

with open("rules.txt", "a") as f:
    for name in data:
        f.write("%s\n" % name)
