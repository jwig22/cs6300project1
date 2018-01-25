# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

actions = None

class SearchNode:
    def __init__(self, s, parent=None, parent_action=None, cost=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.parent_action = parent_action
        self.state = s[:]
        #self.actions = A

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' '+str(self.parent)+' '+str(self.parent_action)+' '+str(self.cost)


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def backpath(node):
    #print 'Goal Node:',node
    path = []
    plan = []
    while node != None:
        path.append(node.state)
        plan.append(node.parent_action)
        node = node.parent
    plan.pop()
    plan.reverse()
    #print 'plan',plan
    return plan

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    visited_list = []
    ##
    SN_visited_list = []
    node_frontier = util.Stack()
    SN_init = SearchNode(problem.getStartState())
    node_frontier.push(SN_init)
    ##
    frontier = util.Stack() #Define the frontier as a stack (LIFO)
    x_0 = (problem.getStartState(),None,0)
    frontier.push(x_0)   #push the initial node to the frontier
    while frontier.isEmpty() is False:
        cur_node = frontier.pop()   #pull last node off of the stack
        cur_state = cur_node[0]
        ##
        SN_cur_node = node_frontier.pop()
        SN_cur_state = SN_cur_node.state
        if SN_cur_state not in SN_visited_list:
            SN_visited_list.append(SN_cur_state)
        ##
        if cur_state not in visited_list:   #if current state has not been visited, add it
            visited_list.append(cur_state)
            #check to see if it's the goal
            if problem.isGoalState(cur_state):  #returns true if cur_state == goal
                plan = backpath(SN_cur_node)
                return plan
            else:   #cur_node is not the goal, find children and add them to the frontier
                children = problem.getSuccessors(cur_state)
                for child in children:
                    ##
                    node_new = SearchNode(child[0],SN_cur_node,child[1])
                    node_frontier.push(node_new)
                    ##
                    frontier.push(child)
    print 'frontier emptied'
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited_list = []
    ##
    SN_visited_list = []
    node_frontier = util.Queue()
    SN_init = SearchNode(problem.getStartState())
    node_frontier.push(SN_init)
    ##
    frontier = util.Queue()  # Define the frontier as a stack (LIFO)
    x_0 = (problem.getStartState(), None, 0)
    frontier.push(x_0)  # push the initial node to the frontier
    while frontier.isEmpty() is False:
        cur_node = frontier.pop()  # pull last node off of the stack
        cur_state = cur_node[0]
        ##
        SN_cur_node = node_frontier.pop()
        SN_cur_state = SN_cur_node.state
        if SN_cur_state not in SN_visited_list:
            SN_visited_list.append(SN_cur_state)
        ##
        if cur_state not in visited_list:  # if current state has not been visited, add it
            visited_list.append(cur_state)
            # check to see if it's the goal
            if problem.isGoalState(cur_state):  # returns true if cur_state == goal
                plan = backpath(SN_cur_node)
                return plan
            else:  # cur_node is not the goal, find children and add them to the frontier
                children = problem.getSuccessors(cur_state)
                for child in children:
                    ##
                    node_new = SearchNode(child[0], SN_cur_node, child[1])
                    node_frontier.push(node_new)
                    ##
                    frontier.push(child)
    print 'frontier emptied'
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited_list = []
    cost = 0.0
    ##
    SN_visited_list = []
    node_frontier = util.PriorityQueue()
    SN_init = SearchNode(problem.getStartState())
    node_frontier.push(SN_init,cost)
    ##
    frontier = util.PriorityQueue()  # Define the frontier as a stack (LIFO)
    x_0 = (problem.getStartState(), None, 0.0)
    frontier.push(x_0,cost)  # push the initial node to the frontier
    while frontier.isEmpty() is False:
        cur_node = frontier.pop()  # pull last node off of the stack
        cur_state = cur_node[0]
        ##
        SN_cur_node = node_frontier.pop()
        SN_cur_state = SN_cur_node.state
        cur_cost = SN_cur_node.cost
        if SN_cur_state not in SN_visited_list:
            SN_visited_list.append(SN_cur_state)
        ##
        if cur_state not in visited_list:  # if current state has not been visited, add it
            visited_list.append(cur_state)
            # check to see if it's the goal
            if problem.isGoalState(cur_state):  # returns true if cur_state == goal
                plan = backpath(SN_cur_node)
                return plan
            else:  # cur_node is not the goal, find children and add them to the frontier
                children = problem.getSuccessors(cur_state)
                for child in children:

                    cost_new = float(cur_cost+child[2])
                    ##
                    node_new = SearchNode(child[0], SN_cur_node, child[1],cost_new)
                    node_frontier.push(node_new,cost_new)
                    ##
                    frontier.push(child,cost_new)
    print 'frontier emptied'
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited_list = []
    cost = 0.0
    ##
    SN_visited_list = []
    node_frontier = util.PriorityQueue()
    SN_init = SearchNode(problem.getStartState())
    node_frontier.push(SN_init, cost)
    ##
    frontier = util.PriorityQueue()  # Define the frontier as a stack (LIFO)
    x_0 = (problem.getStartState(), None, 0.0)
    frontier.push(x_0, cost)  # push the initial node to the frontier
    while frontier.isEmpty() is False:
        cur_node = frontier.pop()  # pull last node off of the stack
        cur_state = cur_node[0]
        ##
        SN_cur_node = node_frontier.pop()
        SN_cur_state = SN_cur_node.state
        cur_cost = SN_cur_node.cost
        if SN_cur_state not in SN_visited_list:
            SN_visited_list.append(SN_cur_state)
        ##
        if cur_state not in visited_list:  # if current state has not been visited, add it
            visited_list.append(cur_state)
            # check to see if it's the goal
            if problem.isGoalState(cur_state):  # returns true if cur_state == goal
                plan = backpath(SN_cur_node)
                return plan
            else:  # cur_node is not the goal, find children and add them to the frontier
                children = problem.getSuccessors(cur_state)
                for child in children:
                    #We will need to calculate a new cost here
                    cost_new = float(cur_cost + child[2])
                    ##
                    node_new = SearchNode(child[0], SN_cur_node, child[1], cost_new)
                    node_frontier.push(node_new, cost_new)
                    ##
                    frontier.push(child, cost_new)
    print 'frontier emptied'
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
