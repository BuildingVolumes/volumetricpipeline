from random import random
import json
from BackEnd.contentToJson import contentToJson
def randValue(identifer, value):
    result = random() * value
    dictonary = {
        0: result
    }
    contentToJson(identifer, dictonary)