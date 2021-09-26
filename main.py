# imports
import copy

################ global variables ################
end_pos = (12, 19)


################ node class for gbfs ################
class gbfsNode:
    def __init__(self, board, parent, operator, steps):
        self.board = board
        self.parent = parent
        self.operator = operator
        self.steps = steps
        self.mht_dist = 0

        self.mht_dist = get_manhattan_dist(self.board)

    def __eq__(self, other):
        return (type(self) == type(other)) and (self.board == other.board)

    def __lt__(self, other):
        return self.mht_dist < other.mht_dist


def create_gbfs_node(board, parent, operator, steps):
    return gbfsNode(board, parent, operator, steps)


################ node class for A* ################
class astarNode:
    def __init__(self, board, parent, operator, g, h, f):
        self.board = board
        self.parent = parent
        self.operator = operator
        self.g = g  # distance to start node
        self.h = h  # distance to goal node
        self.f = f  # total cost

    def __eq__(self, other):
        return (type(self) == type(other)) and (self.board == other.board)

    def __lt__(self, other):
        return self.f < other.f


def create_astar_node(board, parent, operator, g, h, f):
    return astarNode(board, parent, operator, g, h, f)


# calculating heuristic
def get_manhattan_dist(board):
    agent_x, agent_y = get_agent_position(board)
    end_x, end_y = end_pos[0], end_pos[1]

    dist = abs(agent_x - end_x) + abs(agent_y - end_y)
    return dist


################ utility functions ################
def print_board(board, board_size):
    for i in range(board_size):
        for j in range(board_size):
            print(board[i][j], end=' ')
        print()


def get_agent_position(board):
    for r_index, row in enumerate(board):
        for c_index, col in enumerate(row):
            if 'A' in col:
                return r_index, c_index


################ expansion ################
def expand_node(node, board_size, algo):
    expanded_nodes = []

    up_board = move_up(node.board, board_size)
    left_board = move_left(node.board, board_size)
    right_board = move_right(node.board, board_size)
    down_board = move_down(node.board, board_size)

    if up_board is not None:
        if algo == "gbfs":
            up_node = create_gbfs_node(up_board, node, "up", node.steps + 1)
        elif algo == "astar":
            g = node.g + 1
            h = get_manhattan_dist(up_board)
            f = g + h
            up_node = create_astar_node(up_board, node, "up", g, h, f)
        expanded_nodes.append(up_node)

    if left_board is not None:
        if algo == "gbfs":
            left_node = create_gbfs_node(left_board, node, "left", node.steps + 1)
        elif algo == "astar":
            g = node.g + 1
            h = get_manhattan_dist(left_board)
            f = g + h
            left_node = create_astar_node(left_board, node, "left", g, h, f)
        expanded_nodes.append(left_node)

    if right_board is not None:
        if algo == "gbfs":
            right_node = create_gbfs_node(right_board, node, "right", node.steps + 1)
        elif algo == "astar":
            g = node.g + 1
            h = get_manhattan_dist(right_board)
            f = g + h
            right_node = create_astar_node(right_board, node, "right", g, h, f)
        expanded_nodes.append(right_node)

    if down_board is not None:
        if algo == "gbfs":
            down_node = create_gbfs_node(down_board, node, "down", node.steps + 1)
        elif algo == "astar":
            g = node.g + 1
            h = get_manhattan_dist(down_board)
            f = g + h
            down_node = create_astar_node(down_board, node, "down", g, h, f)
        expanded_nodes.append(down_node)

    return expanded_nodes


################ movement functions ################
def move_left(board, board_size):
    move_board = copy.deepcopy(board)
    x, y = get_agent_position(board)

    if (y > 0) and (board[x][y - 1] == ' '):
        move_board[x][y], move_board[x][y - 1] = move_board[x][y - 1], move_board[x][y]
        return move_board

    return None


def move_right(board, board_size):
    move_board = copy.deepcopy(board)
    x, y = get_agent_position(board)

    if (y < board_size - 1) and (board[x][y + 1] == ' '):
        move_board[x][y], move_board[x][y + 1] = move_board[x][y + 1], move_board[x][y]
        return move_board

    return None


def move_up(board, board_size):
    move_board = copy.deepcopy(board)
    x, y = get_agent_position(board)

    if (x > 0) and (board[x - 1][y] == ' '):
        move_board[x][y], move_board[x - 1][y] = move_board[x - 1][y], move_board[x][y]
        return move_board

    return None


def move_down(board, board_size):
    move_board = copy.deepcopy(board)
    x, y = get_agent_position(board)

    if (x < board_size - 1) and (board[x + 1][y] == ' '):
        move_board[x][y], move_board[x + 1][y] = move_board[x + 1][y], move_board[x][y]
        return move_board

    return None


################ greedy best first search code ################
def greedy_bfs(start, board_size):
    # get agent index
    agent_pos = get_agent_position(start)

    # compare with goal position
    if agent_pos == end_pos:
        return [None], 0

    open_list = []
    closed_list = []

    # adding starting state to open list
    start_node = create_gbfs_node(start, None, None, 0)
    open_list.append(start_node)

    # no of moves
    count = 1

    # while there are nodes to expand
    while open_list:
        open_list.sort()

        # lowest cost node
        current_node = open_list.pop(0)
        closed_list.append(current_node)

        # coordinates of agent
        agent_pos = get_agent_position(current_node.board)

        count += 1

        # termination check
        if agent_pos == end_pos:
            return current_node, count

        # list of neighbours / successors
        successors = expand_node(current_node, board_size, "gbfs")

        # successor only explored if not in open / closed
        for node in successors:
            if (node not in open_list) and (node not in closed_list):
                open_list.append(node)

    # no solution found
    return None, 0


################ A* search code ################
def a_star(start, board_size):
    # get agent index
    agent_pos = get_agent_position(start)

    # compare with goal position
    if agent_pos == end_pos:
        return [None], 0

    open_list = []
    closed_list = []

    # adding starting state to open list
    start_node = create_astar_node(start, None, None, 0, 0, 0)
    start_node.h = get_manhattan_dist(start_node.board)
    start_node.f = start_node.g + start_node.h

    open_list.append(start_node)

    # of moves
    count = 1

    # while there are nodes to expand
    while open_list:
        open_list.sort()

        # lowest cost node
        current_node = open_list.pop(0)
        closed_list.append(current_node)

        #  coordinates of agent
        agent_pos = get_agent_position(current_node.board)

        count += 1

        # termination check
        if agent_pos == end_pos:
            return current_node, count

        # list of neighbours / successors
        successors = expand_node(current_node, board_size, "astar")

        # successor only explored if not in open / closed
        for node in successors:
            if node in closed_list:
                continue

            # check that successor has lower cost and is not already in open list
            if open_check(open_list, node):
                open_list.append(node)

    # no solution found
    return None, 0


# Check if a neighbor should be added to open list
def open_check(open_list, neighbor):
    for node in open_list:
        if neighbor == node and neighbor.f >= node.f:
            return False
    return True


################ driver code ################
def main():
    board_size = 20

    print("\nBoard size: ", board_size)

    board = [
        ['*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*'],  # 0
        ['*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*'],  # 1
        ['*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*'],  # 2
        ['*', '*', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '*', '*', ' ', ' ', ' ', ' ', ' ', ' ', ' '],  # 3
        ['*', '*', ' ', '*', '*', '*', '*', '*', '*', '*', ' ', '*', '*', ' ', '*', '*', '*', ' ', '*', '*'],  # 4
        ['*', '*', ' ', '*', '*', '*', '*', '*', '*', '*', ' ', ' ', ' ', ' ', '*', '*', '*', ' ', ' ', ' '],  # 5
        ['*', '*', ' ', '*', '*', '*', '*', '*', '*', '*', ' ', '*', '*', ' ', '*', '*', '*', ' ', '*', ' '],  # 6
        ['*', '*', ' ', ' ', ' ', ' ', '*', '*', '*', '*', ' ', '*', '*', ' ', ' ', ' ', ' ', ' ', '*', ' '],  # 7
        ['*', '*', '*', ' ', '*', '*', '*', '*', '*', '*', ' ', '*', '*', '*', '*', '*', '*', ' ', '*', ' '],  # 8
        ['*', '*', '*', ' ', '*', '*', '*', '*', ' ', ' ', ' ', '*', '*', '*', '*', '*', '*', '*', '*', ' '],  # 9
        ['*', '*', '*', ' ', '*', ' ', ' ', ' ', ' ', '*', ' ', '*', '*', '*', '*', '*', ' ', ' ', ' ', ' '],  # 10
        ['*', '*', ' ', ' ', '*', ' ', '*', '*', ' ', '*', ' ', ' ', ' ', ' ', ' ', '*', ' ', '*', '*', '*'],  # 11
        ['*', '*', ' ', '*', ' ', ' ', '*', '*', ' ', '*', '*', '*', '*', '*', ' ', '*', ' ', ' ', ' ', ' '],  # 12
        ['*', '*', ' ', '*', ' ', ' ', ' ', '*', ' ', '*', '*', '*', '*', '*', ' ', '*', ' ', '*', '*', '*'],  # 13
        ['A', ' ', ' ', ' ', '*', '*', ' ', '*', ' ', '*', '*', '*', '*', '*', ' ', '*', '*', '*', ' ', ' '],  # 14
        ['*', '*', ' ', '*', '*', '*', ' ', '*', ' ', '*', '*', '*', '*', '*', ' ', ' ', ' ', ' ', ' ', '*'],  # 15
        ['*', '*', ' ', ' ', ' ', ' ', ' ', '*', ' ', '*', '*', '*', '*', '*', ' ', '*', '*', ' ', '*', '*'],  # 16
        ['*', '*', '*', '*', '*', '*', '*', '*', ' ', '*', '*', '*', '*', '*', '*', '*', '*', ' ', '*', '*'],  # 17
        ['*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*'],  # 18
        ['*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*', '*'],  # 19
    ]

    print_board(board, board_size)

    ########## GBFS ###########
    print("\n------------------ GBFS ---------------------\n")

    result, moves = greedy_bfs(board, board_size)

    if result == [None]:
        print("\nAgent already at the goal position!")

    elif result is None:
        print("\nNo solution found.")

    else:
        print("\n_________   GBFS End State    _________\n")
        print_board(result.board, board_size)
        print("\nAlgorithm used: Greedy Best First Search :")
        print("\tNumber of moves utilised: ", moves)
        print("\tPath cost: ", result.steps)

        opt = input("\nPress 1 to view moves from start to end, or any other key to continue to A*: ")
        if (opt == '1'):
            node_seq = []
            node_seq.append(result)
            current_node = result

            while (current_node.parent is not None):
                parent_node = current_node.parent
                prev_node = parent_node

                node_seq.append(prev_node)
                current_node = parent_node

            node_seq.reverse()

            print("\nSequence from start -> goal:\n\tAgent initially at index (14, 0)")

            for node in node_seq:
                if (node.operator is not None):
                    pos = get_agent_position(node.board)
                    print("\tAgent moves " + node.operator + " to index " + str(pos))

    print("\n---------------------------------------\n")

    ########## A* ###########
    print("\n------------------ A* ---------------------\n")

    result, moves = a_star(board, board_size)

    if result == [None]:
        print("\nAgent already at the goal position!")

    elif result is None:
        print("\nNo solution found.")

    else:
        print("\n_________   A* End State    _________\n")
        print_board(result.board, board_size)
        print("\nAlgorithm used: A* :")
        print("\tNumber of moves utilised: ", moves)
        print("\tPath cost: ", result.g)

        opt = input("\nPress 1 to view moves from start to end, or any other key to terminate: ")
        if (opt == '1'):
            node_seq = []
            node_seq.append(result)
            current_node = result

            while (current_node.parent is not None):
                parent_node = current_node.parent
                prev_node = parent_node

                node_seq.append(prev_node)
                current_node = parent_node

            node_seq.reverse()

            print("\nSequence from start -> goal:\n\tAgent initially at index (14, 0)")

            for node in node_seq:
                if (node.operator is not None):
                    pos = get_agent_position(node.board)
                    print("\tAgent moves " + node.operator + " to index " + str(pos))


# execute program
if __name__ == "__main__":
    main()
