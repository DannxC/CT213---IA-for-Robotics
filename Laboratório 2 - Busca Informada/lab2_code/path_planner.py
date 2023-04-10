from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Reset grid
        self.node_grid.reset()

		# Setup start and goal nodes
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        start_node.f = 0
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
            
        # Validate initial and goal nodes
        if start_node is None or goal_node is None:
            return [], inf
        
        # Priority Queue
        pq = []
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            f, node = heapq.heappop(pq)
            node.closed = True

            # Goal Node finded
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            
            # Try every successors
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(successor[0], successor[1])
                new_f = f + self.cost_map.get_edge_cost(node.get_position(), successor_node.get_position())
                if new_f < successor_node.f and not successor_node.closed:
                    successor_node.f = new_f
                    successor_node.parent = node
                    heapq.heappush(pq, (new_f, successor_node))

		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        return [], inf
    
    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Reset grid
        self.node_grid.reset()

		# Setup start and goal nodes
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_node.f = 0

        # Validate initial and goal nodes
        if start_node is None or goal_node is None:
            return [], inf
        
        # Priority Queue
        pq = []
        heapq.heappush(pq, (start_node.distance_to(*goal_position), start_node))    # Heurística

        while pq:
            h, node = heapq.heappop(pq)
            node.closed = True

            # Goal Node finded
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            
            # Try every successors
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(successor[0], successor[1])
                successor_h = successor_node.distance_to(*goal_position)   # Heurística
                if successor_h < h and not successor_node.closed:
                    edge_cost = self.cost_map.get_edge_cost(node.get_position(), successor_node.get_position())
                    successor_node.f = node.f + edge_cost
                    successor_node.parent = node
                    heapq.heappush(pq, (successor_h, successor_node))

		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        return [], inf
    
    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """ 
        # Reset grid
        self.node_grid.reset()

		# Start and End nodes
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_node.g = 0
        start_node.f = start_node.g + start_node.distance_to(*goal_position)    # f = g + h
        
        # Validate initial and goal nodes
        if start_node is None or goal_node is None:
            return [], inf
            
        # Priority Queue
        pq = []
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            _, node = heapq.heappop(pq)
            node.closed = True

            # Goal Node finded
            if node == goal_node:
                return self.construct_path(goal_node), goal_node.f
            
            # Try every successors
            for successor in self.node_grid.get_successors(*node.get_position()):
                successor_node = self.node_grid.get_node(successor[0], successor[1])
                
                h = successor_node.distance_to(*goal_position)  # Heurística
                edge_cost = self.cost_map.get_edge_cost(node.get_position(), successor_node.get_position())
                new_f = node.g + edge_cost + h

                if successor_node.f > new_f and not successor_node.closed:
                    successor_node.g = node.g + edge_cost
                    successor_node.f = new_f
                    successor_node.parent = node
                    heapq.heappush(pq, (new_f, successor_node))

		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        return [], inf
