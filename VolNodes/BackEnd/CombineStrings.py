from BackEnd.contentToJson import contentToJson

def combineStrings(identifer, values):
    dictonary = {0: values, 1: len(values)}
    contentToJson(identifer, dictonary)