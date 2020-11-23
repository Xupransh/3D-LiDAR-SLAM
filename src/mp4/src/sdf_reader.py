import xml.etree.ElementTree as ET
import pickle


#store coordinates in a list
#[(x,y),...]
obstacle_list = []


tree = ET.parse("../models/eceb/model.sdf")
root = tree.getroot()

model = root[0]
print (len(model))
angles = set()
x_angle = set([0.0, 0.5, -0.5, 0.3, -0.3, 3.1, -3.1])

test_list = []

for n in range(1,len(model)-1):
    link = model[n]
    dimension_str = link[0][0][0][0].text
    parse_dim = dimension_str.split(' ')
    x_dim = float(parse_dim[0])
    y_dim = float(parse_dim[1])

    position_str = link[2].text
    parse_pos = position_str.split(' ')
    x_pos = float(parse_pos[0])+2
    y_pos = float(parse_pos[1])-4
    angle = float(parse_pos[5])
    angle = round(angle, 1)
    angles.add(angle)

    flag = False
    if x_pos == 15 and y_pos == -1:
        flag = True

    # check the wall is on X or Y axis
    if angle in x_angle:
        for i in range(int((x_pos-(x_dim/2+1))), int((x_pos+(x_dim/2)))):
            for j in range(int((y_pos-(y_dim/2+1))), int((y_pos+(y_dim/2+1)))):
                obstacle_list.append((i,j))
                if flag:
                    test_list.append((i,j))
    else:
        for j in range(int((y_pos-(x_dim/2+1))), int((y_pos+(x_dim/2+1)))):
            for i in range(int((x_pos-(y_dim/2+1))), int((x_pos+(y_dim/2)))):
                obstacle_list.append((i,j))
                if flag:
                    test_list.append((i,j))


print (angles)
print(test_list)

with open('obstacle_list.data', 'wb') as filehandle:
    # store the data as binary data stream
    pickle.dump(obstacle_list, filehandle)

with open('obstacle_list.data', 'rb') as filehandle:
    # read the data as binary data stream
    obstacle_list = pickle.load(filehandle)

print(len(obstacle_list))
