import math
from matplotlib import pyplot as plt
import numpy as np

class Util:
    @staticmethod
    def dist(n1, n2):
        return math.sqrt((n1.pos[0] - n2.pos[0])**2 + (n1.pos[1] - n2.pos[1])**2)
    
    @staticmethod
    def angle(n1, n2):
        return math.atan2(n2.pos[1] - n1.pos[1], n2.pos[0] - n1.pos[0])


class Node:
    def __init__(self, pos, cost=0):
        self.pos = pos
        self.cost = cost

class Graph:
    def __init__(self):
        self.vertices = []
        self.edges = []
        self.edge_lengths = []

    def connect_new_node(self, parent, new_node):
        # calculate the cost of the new node
        new_node.cost = parent.cost + Util.dist(parent, new_node)

        # connect the new node to the graph
        self.vertices.append(new_node)
        self.edges.append((parent, new_node))
        self.edge_lengths.append(Util.dist(parent, new_node))

    def nodes_within_radius(self, node, radius):
        nodes = []
        for vertex in self.vertices:
            if Util.dist(node, vertex) < radius:
                nodes.append(vertex)
        return nodes
    
    def rewire(self, node, new_parent):
        # rewire the graph to connect the node to the new parent
        # remove the old edge
        for i in range(len(self.edges)):
            if self.edges[i][1] == node:
                self.edges.pop(i)
                self.edge_lengths.pop(i)
                break
        
        # add the new edge
        self.connect_new_node(new_parent, node)


    

    

class RRTStar:

    def __init__(self, startpos, endpos, map, n_iter, step_size, neighbor_radius, goal_radius, collision_radius):
        self.startpos = startpos
        self.endpos = endpos
        self.map = map
        self.n_iter = n_iter
        self.step_size = step_size
        self.neighbor_radius = neighbor_radius

        self.goal_radius = goal_radius

        self.collision_radius = collision_radius

        self.G = Graph()

        self.G.vertices.append(Node(startpos))
        # self.G.vertices.append(Node(endpos))

    def random_position(self):
        # generate a random position within the map
        # return Node((np.random.uniform(0, self.map.shape[0]), np.random.uniform(0, self.map.shape[1])))
        # generate a random position within rectangle of start and goal
        return  Node((np.random.uniform(min(self.startpos[0], self.endpos[0]), max(self.startpos[0], self.endpos[0])), np.random.uniform(min(self.startpos[1], self.endpos[1]), max(self.startpos[1], self.endpos[1]))))
    
    
    def nearest(self, node):
        # find the nearest node in the graph to the given node
        nearest_node = self.G.vertices[0]
        for vertex in self.G.vertices:
            if Util.dist(node, vertex) < Util.dist(node, nearest_node):
                nearest_node = vertex
        return nearest_node
    
    def new_vertex(self, parent, target):
        # generate a new vertex from the parent to the target
        angle = Util.angle(parent, target)
        return Node((parent.pos[0] + self.step_size * math.cos(angle), parent.pos[1] + self.step_size * math.sin(angle)))
    
    def collision_free(self, node1, node2):
        # check if there is a occupied cell between node1 and node2, add a buffer to the collision radius
    
        angle = Util.angle(node1, node2)
        dist = Util.dist(node1, node2)
        # collision buffer = self.collision_radius
        for i in range(0, int(dist), 1):
            x = int(node1.pos[0] + i * math.cos(angle))
            y = int(node1.pos[1] + i * math.sin(angle))
            # if there is an occupied cell in collision buffer, return false
            for j in range(-self.collision_radius, self.collision_radius, 1):
                for k in range(-self.collision_radius, self.collision_radius, 1):
                    if self.map[y+j][x+k] <= 255 * 0.196:
                        return False
        return True
    


        # for i in range(0, int(dist), 1):
        #     x = int(node1.pos[0] + i * math.cos(angle))
        #     y = int(node1.pos[1] + i * math.sin(angle))
        #     if self.map[y][x] <= 255 * 0.196:
        #         return False
        # return True
    
    def goal_reached(self, node):
        # check if the node is within the goal radius
        return Util.dist(node, Node(self.endpos)) < self.goal_radius
    
    def rewire(self, new_node):
        # rewire the graph to find a better path to the new node
        neighbors = self.G.nodes_within_radius(new_node, self.neighbor_radius)
        # find the best new parent form the neighbors
        best_parent = neighbors[0]
        for neighbor in neighbors:
            if Util.dist(neighbor, new_node) + neighbor.cost < best_parent.cost:
                best_parent = neighbor
        # rewire the graph
        self.G.rewire(new_node, best_parent)
        print('rewire')        

    def backtarce(self, node):
        # backtrace the path from the goal to the start
        path = []
        # use G.edges to find the parent of the node
        for edge in self.G.edges:
            if edge[1] == node:
                path.append(edge[0])
                path.append(edge[1])
                break
        while path[-1] != self.G.vertices[0]:
            for edge in self.G.edges:
                if edge[1] == path[-1]:
                    path.append(edge[0])
                    break
        return path



# main
if __name__ == '__main__':
    def read_pgm_2(path):
        pgmf = open(path, 'rb')
        im = plt.imread(pgmf)
        raster = np.array(im)
        pgmf.close()
        return raster

    # read the map as a grayscale image

    raster = read_pgm_2('map.pgm')

    print(raster.shape)



    # visualize the map in plt
    plt.imshow(raster, cmap='gray')


    planning = RRTStar((170,213), (209,146), raster, 5000, step_size=2, neighbor_radius=5, goal_radius=2, collision_radius=3)

    # draw the start and goal locations
    plt.plot(planning.startpos[0], planning.startpos[1], 'ro')
    plt.plot(planning.endpos[0], planning.endpos[1], 'ro')
    plt.draw()
    plt.pause(0.001)

    for i in range(planning.n_iter):
        random_node = planning.random_position()

        print(random_node.pos)

        nearest_node = planning.nearest(random_node)

        print(nearest_node.pos, i)

        new_node = planning.new_vertex(nearest_node, random_node)

        if planning.collision_free(nearest_node, new_node):
            planning.G.connect_new_node(nearest_node, new_node)
        else:
            continue

        if planning.goal_reached(new_node):
            print('goal reached')
            path = planning.backtarce(new_node)
            break

        planning.rewire(new_node)

        print(new_node.pos)
        # plt.plot(new_node.pos[0], new_node.pos[1], 'bo')
        # # draw the newest edge
        # plt.plot([planning.G.edges[-1][0].pos[0], planning.G.edges[-1][1].pos[0]], [planning.G.edges[-1][0].pos[1], planning.G.edges[-1][1].pos[1]], 'b-', linewidth=0.5)
        # plt.draw()
        # plt.pause(0.001)
    # keep the plot open

        # draw the vertices
    for vertex in planning.G.vertices:
        plt.plot(vertex.pos[0], vertex.pos[1], 'bo', markersize=1)
    
    # draw the edges
    if len(planning.G.edges) > 0:
        for edge in planning.G.edges:
            plt.plot([edge[0].pos[0], edge[1].pos[0]], [edge[0].pos[1], edge[1].pos[1]], 'b-')

    plt.plot(planning.startpos[0], planning.startpos[1], 'ro')
    plt.plot(planning.endpos[0], planning.endpos[1], 'ro')

    # draw the path
    for i in range(len(path)-1):
        plt.plot([path[i].pos[0], path[i+1].pos[0]], [path[i].pos[1], path[i+1].pos[1]], 'r-', linewidth=2)


    plt.show()
    plt.show()





        









