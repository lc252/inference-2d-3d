# for files that are formatted in mm not m

filename = "hp_mouse"

with open(f"/home/fif/lc252/inference-2d-3d/src/object_detection/obj_models/{filename}.obj", "r") as f:
    data = f.read().split('\n')
    text = ""
    for line in data:
        temp = line.split(" ")
        if temp[0] == 'v':
            temp[1:4] = [float(i)/1000 for i in temp[1:4]]

        temp = str(temp)
        temp = temp.replace('[','')
        temp = temp.replace(']','')
        temp = temp.replace('\'','')
        temp = temp.replace(',','')
        text += temp + '\n'
    
with open(f"/home/fif/lc252/inference-2d-3d/src/object_detection/obj_models/{filename}_scaled.obj", "w") as f:
    f.write(text)