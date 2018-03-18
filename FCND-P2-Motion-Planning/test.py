with open('colliders.csv') as f:
    first_line = f.readline()
    lat0 = first_line.split(',')[0].strip().split(' ')[1]
    lon0 = first_line.split(',')[1].strip().split(' ')[1]

print(lat0, lon0)