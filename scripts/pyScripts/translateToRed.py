import json
pathpath = input()
with open(pathpath, 'r') as f:
    wps = json.loads(f.read())

for i in range(len(wps['waypoints'])):
    for point in ('anchorPoint', 'prevControl', 'nextControl'):
        if wps['waypoints'][i][point] == None:
            continue
        wps['waypoints'][i][point]['y'] = 8.02 - wps['waypoints'][i][point]['y']

with open('.'.join(pathpath.split('.')[:-1]) + ' red.path', 'w') as f:
    f.write(json.dumps(wps))
