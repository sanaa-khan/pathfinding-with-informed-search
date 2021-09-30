# Pathfinding With Informed Search Algorithms
Python implementation of a graph-based search agent, using informed search algorithms - greedy search and A*.

## Problem Statement
Consider you have a Maze that needs to be solved. The goal of an agent is to complete the Maze in fixed number of moves. Your implementation should be modular so that the sensors, actuators and environment characteristics (size etc.) can be easily changed. You need to implement a graph based search agent with visited state check. You are required to formulate this problem by defining the four parameters:

- Initial State
- Successor
- Goal Test
- Path cost

Note: Moves can be made only to an empty block represented by 1 while the filled block can be considered as 0. Sequence of steps should be in given order: Up, Left, Right and then Down.

## Problem Description
- Available actions are Left, Right, Up, Down.
- The termination criterion is that the Maze is completed by the agent.
- You need to solve this Maze using BFS and DFS algorithm.

Environment announces when this criterion is met and stops the execution.

- Agent knows the size of Maze (grid 12 x 12), the content of the cell they land in and the location of the landing cell (coordinates).
- The agent should be initially placed at 5 x 12 block as described in the Maze grid.
- The performance of an agent is calculated after the termination criteria is met. The performance measure of an agent is the (number of steps used) to complete the maze. Maze is completed when agent reaches 11 x 1 block.
- The environment is deterministic and fully observable.
- The perception is given by the environment and includes, cell coordinates and if the current piece in the cell is empty or blocked.

## Contact
Need help? Got any queries? Feel free to hit me up at sanakahnn@gmail.com.
