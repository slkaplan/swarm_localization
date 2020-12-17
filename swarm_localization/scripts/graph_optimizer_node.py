import numpy as np
import g2o
import os
import pickle
from collate_data import IndividualVertex

def get_data(filename):
    with open(filename,'rb') as f:
        data = pickle.load(f)
        return data

class PoseGraphOptimization(g2o.SparseOptimizer):
    """
    This class uses g2o's python binding to create an optimizer object. 
    Nominally, it needs a pickled file which contains a dictionary of custom IndividualVertex Objects. 
    
    Author: qihang@outlook.com
    Editors: Luke Nonas-Hunter & Sam Kaplan & Emma Pan & Eamon O'Brien
    """

    def __init__(self):
        # inherits traits from g2o's Sparse Optimizer class
        super().__init__()

        # intitialize solver and optimizer
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        """
        Adds a vertex to an optimizer object. 
        id: integer representing vertex number
        pose: g2o.Isometry3d object
        """
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement, information=np.identity(6), robust_kernel=None):
        """
        Adds an edge between two vertices to an optimizer object. 
        vertices: array of length two containing the id's of the vertices you wish to link
        measurement: g2o.Isometry3d object corresponding to the translation between vertices
        """
        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()

    def make_vertices_and_edges(self, data):
        """
        Adds all vertices and edges to an optimizer object from data. 
        data: pickle file containing a dictionary of IndividualVertex objects. See collate_data.py for more details
        """

        # loops through dict and makes all vertices
        for key in data:
            ind = data[key].index

            # vertex pose
            x = data[key].x
            y = data[key].y

            arr = np.array(x,y,0) # using a 3D isometry to represent 2D data, so 3rd dimension is 0
            t = g2o.Isometry3d(np.identity(3), arr) # needs an indentity matrix to make the 3D object happy
            self.add_vertex(ind, t)
        
        vertices = super().vertices()


        # need to add edges after vertices, so we have to loop through the same data twice 
        for vertex in data.values(): 
            ind = vertex.index # id of current vertex

            for index, edge in vertex.edge.items(): # index is id of vertex the current vertex has an edge to through visual odometry

                # need to convert our 2D data to 3D again
                arr = np.array(edge[0], edge[1], 0)
                diff = g2o.Isometry3d(np.identity(3), arr)

                self.add_edge([ind, index], diff)

        print('num vertices:', len(super().vertices()))
        print('num edges: ', len(super().edges()))

if __name__ == '__main__':
    # gets data from pickle file
    data = get_data('test.pickle')

    # initializers optimizer object
    opt = PoseGraphOptimization()

    # calls method to add vertices and edges to optimizer object
    opt.make_vertices_and_edges(data)

    # performs optimization
    opt.optimize()

    # saves optimized data to a .g2o file
    opt.save('post_opt.g2o')