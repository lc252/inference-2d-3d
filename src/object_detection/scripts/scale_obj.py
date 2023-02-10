# for files that are formatted in mm not m

filename = "model_car"

with open(f"/home/fif/lc252/inference-2d-3d/src/object_detection/model_geometry/{filename}.obj", "r") as f:
    data = f.read().split('\n')
    text = ""
    for line in data:
        temp = line.split(" ")
        if temp[0] == 'v':
            temp[1:4] = [float(i)/(1000*34) for i in temp[1:4]]
        else:
            continue

        temp = str(temp)
        temp = temp.replace('[','')
        temp = temp.replace(']','')
        temp = temp.replace('\'','')
        temp = temp.replace(',','')
        text += temp + '\n'
    
with open(f"/home/fif/lc252/inference-2d-3d/src/object_detection/model_geometry/{filename}_scaled.obj", "w") as f:
    f.write(text)