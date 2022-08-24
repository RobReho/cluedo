# -*- coding: utf-8 -*-
import random

people_list = ["Scarlet","Plum","Green","White","Peacock","Mustard"]
weapons_list = ["candlestik","rope","lead_pipe","revolver","spanner","dagger"]
places_list = ["library","conservatory","lounge","ballroom","billiard_room","kitchen","dining_room","hall","study"]

ID0 = []
ID1 = []
ID2 = []
ID3 = []
source_ID = [ID0,ID1,ID2,ID3]

hints = people_list + weapons_list + places_list
random.shuffle(hints)

while hints:
    for i in range (0, 4):
        if hints:
            source_ID[i].append( hints.pop() )  
        
#for i, target in enumerate([ID0, ID1, ID2, ID3]):
#     for _ in range(i+1):
#         target.append(hints.pop(random.randrange(len(hints))))

print(ID0)
print(ID1)
print(ID2)
print(ID3)
print(hints)