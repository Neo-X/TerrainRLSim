import json
import sys

"""
    python3 data/motions/modifyMocap.py data/motions/humanoid3d_jog_mirror.txt data/motions/humanoid3d_jog_mirror.txt
"""
def addWristData(jsonData):
    
    for row in range(len(jsonData["Frames"])):
        jsonData["Frames"][row].insert(30, 0.0)
        jsonData["Frames"][row].append(0.0)
    
    return jsonData
    
    
if __name__ == '__main__':
    
    input = sys.argv[1]
    out = sys.argv[2]
    f = open(input,"r")
    data = json.load(f)
    data = addWristData(data)
    f.close()
    f = open(out,"w")
    json.dump(data, f, indent=2)
    f.close()
    
    