#!/usr/bin/env python3
import string

def board_to_coord(board_pos):
    gridsize = 20
    if type(board_pos) != str:
        raise TypeError("input should be a string, e.g. A8")
    else:
        x = (ord((board_pos[0].upper())) - ord("A"))*gridsize + gridsize/2
        y = (int(board_pos[1]) - 1)*gridsize + gridsize/2
    return (x,y)

print(board_to_coord("b6"))
