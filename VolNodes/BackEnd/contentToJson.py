import json

def contentToJson(identifer, data):
    with open('./TempFiles/'+identifer+'.json', "w") as outfile:
        json.dump(data, outfile)