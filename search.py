# THIS  CODE  WAS MY OWN WORK , IT WAS  WRITTEN  WITHOUT  CONSULTING  ANY SOURCES  OUTSIDE  OF  THOSE  APPROVED  BY THE  INSTRUCTOR. Gene Lee

# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


# change how fringe is managed given parameter of data structure
def genericSearch(problem, fringe):
    isPQ = isinstance(fringe, util.PriorityQueueWithFunction)
    curr = problem.getStartState()
    if isPQ:
        fringe.push([(curr, "Start", 0)])  # priority queue will store path
        visited = []
    else:
        states = {}  # dict with state as key and (action, parent) as value
        fringe.push(curr)  # non-pq will use dict to generate path
        visited = []

    while not fringe.isEmpty():
        if isPQ:
            path = fringe.pop()
            curr = path[-1][0]
        else:
            curr = fringe.pop()
        if problem.isGoalState(curr):  # exit loop when goal state is found
            break
        if curr not in visited:
            visited.append(curr)
            # graph search requires expanding node only if not visited
            for succsr in problem.getSuccessors(curr):
                if succsr[0] not in visited:  # only add non-visited nodes to fringe
                    if isPQ:
                        newPath = list(path)
                        newPath.append(succsr)
                        fringe.push(newPath)
                    else:
                        fringe.push(succsr[0])
                        # tweaking required because using states dict
                        if isinstance(fringe, util.Stack):
                            states[succsr[0]] = (succsr[1], curr)
                        else:
                            if succsr[0] not in states:  # ensures bfs finds shallowest path
                                states[succsr[0]] = (succsr[1], curr)
                                # states[state] = (action, parent)

    if isPQ:
        path.pop(0)
        return[node[1] for node in path]
    else:
        child = curr  # set child to goal state
        path = []
        visited.pop(0)  # remove start node
        visited.reverse()  # reverse visited list to traverse by parents

        while (True):
            path.append(states[child][0])  # append action
            parent = states[child][1]
            if parent == problem.getStartState(
            ):  # found complete path
                break
            child = parent

        path.reverse()  # reverse since path is backwards
        return path


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    return genericSearch(problem, util.Stack())


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"
    return genericSearch(problem, util.Queue())


def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    def pathCost(path):
        path_ = list(path)
        path_.pop(0)
        # node[1] is the action
        return problem.getCostOfActions([node[1] for node in path_])

    return genericSearch(problem, util.PriorityQueueWithFunction(pathCost))


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    def pathCost(path):
        # path[-1][0] is the state of the last tuple in the path
        heur = heuristic(path[-1][0], problem)
        path_ = list(path)
        path_.pop(0)
        currCost = problem.getCostOfActions([node[1] for node in path_])
        return currCost + heur

    return genericSearch(problem, util.PriorityQueueWithFunction(pathCost))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
