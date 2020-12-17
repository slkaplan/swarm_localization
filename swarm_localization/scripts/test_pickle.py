#!/usr/bin/env python3
import pickle
from collate_data import IndividualVertex
import numpy as np

def get_data(filename):
    with open(filename,'rb') as f:
        data = pickle.load(f)
 
        return data


if __name__ == '__main__':
    data = get_data('test.pickle')
    for vertex in data.values(): 
        
        ind = vertex.index
        print(f"Vertex: {ind}")
        for index, edge in vertex.edge.items():
            print(f"{index}: {edge}")
                