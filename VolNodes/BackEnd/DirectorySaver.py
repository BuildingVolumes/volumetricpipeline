from BackEnd.contentToJson import contentToJson

def directorySaver(identifer, data):
    dictonary = {0: data}
    contentToJson(identifer, dictonary)