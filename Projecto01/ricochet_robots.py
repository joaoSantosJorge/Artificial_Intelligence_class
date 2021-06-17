# ricochet_robots.py: Template para implementação do 1º projeto de Inteligência Artificial 2020/2021.
# Devem alterar as classes e funções neste ficheiro de acordo com as instruções do enunciado.
# Além das funções e classes já definidas, podem acrescentar outras que considerem pertinentes.

# Grupo 71:
# 88077 Bernardo Rosa
# 88079 João Jorge

from search import Problem, Node, astar_search, breadth_first_tree_search, \
    depth_first_tree_search, greedy_search
import sys

import math, copy

class RRState:
    state_id = 0

    def __init__(self, board):
        self.board = board
        self.id = RRState.state_id
        RRState.state_id += 1

    def __lt__(self, other):
        """ Este método é utilizado em caso de empate na gestão da lista
        de abertos nas procuras informadas. """
        return self.id < other.id


class Board:
    """ Representacao interna de um tabuleiro de Ricochet Robots. """

    def __init__(self, N, robot, goal, K, down_barrier, right_barrier):
        self.N = N
        self.robot = robot
        self.goal = goal
        self.K = K
        self.down_barrier = down_barrier
        self.right_barrier = right_barrier
    
    def robot_position(self, robot: str):
        """ Devolve a posição atual do robô passado como argumento. """
        return self.robot[robot]

    # TODO: outros metodos da classe



def parse_instance(filename: str) -> Board:
    """ Lê o ficheiro cujo caminho é passado como argumento e retorna
    uma instância da classe Board. """
    # TODO

    f = open(filename, 'r')
    
    N = int(f.readline())

    robot = {}
    for i in range(4):
        robot_input = f.readline().split()
        robot[robot_input[0]] = (int(robot_input[1]), int(robot_input[2]))

    goal_input = f.readline().split()
    goal = (goal_input[0], int(goal_input[1]), int(goal_input[2]))

    K = int(f.readline())
    
    down_barrier = [[False for i in range(N+1)] for j in range(N)]
    right_barrier = [[False for i in range(N+1)] for j in range(N)]

    for i in range(N):
        down_barrier[i][0] = True
        down_barrier[i][N] = True
        right_barrier[i][0] = True
        right_barrier[i][N] = True

    for i in range(K):
        barrier_input = f.readline().split()
        direction = barrier_input[2]
        if direction=='u':
            down_barrier[int(barrier_input[1])-1][int(barrier_input[0])-1] = True
        elif direction=='r':
            right_barrier[int(barrier_input[0])-1][int(barrier_input[1])] = True 
        elif direction=='d':
            down_barrier[int(barrier_input[1])-1][int(barrier_input[0])] = True
        elif direction=='l':
            right_barrier[int(barrier_input[0])-1][int(barrier_input[1])-1] = True

    f.close()

    
    
    return Board(N, robot, goal, K, down_barrier, right_barrier)


class RicochetRobots(Problem):
    def __init__(self, board: Board):
        """ O construtor especifica o estado inicial. """
        # TODO: self.initial = ...
        self.initial = RRState(board)

    def actions(self, state: RRState):
        """ Retorna uma lista de ações que podem ser executadas a
        partir do estado passado como argumento. """
        # TODO
        board = state.board
        robot_set = set()
        robot_actions = {}
    
        for robot in board.robot:
            robot_set.add(robot)
            robot_actions[robot] = {'u','r','d','l'}
        
        for robot in board.robot:
            robot_set.remove(robot)

            for other_robot in robot_set:
                
                if math.sqrt(pow(board.robot[robot][0]-board.robot[other_robot][0], 2)+pow(board.robot[robot][1]-board.robot[other_robot][1], 2))==1:
                    if board.robot[robot][0] < board.robot[other_robot][0]:
                        robot_actions[robot].discard('d')
                        robot_actions[other_robot].discard('u')
                    elif board.robot[robot][0] > board.robot[other_robot][0]:
                        robot_actions[robot].discard('u')
                        robot_actions[other_robot].discard('d')
                    elif board.robot[robot][1] < board.robot[other_robot][1]:
                        robot_actions[robot].discard('r')
                        robot_actions[other_robot].discard('l')
                    elif board.robot[robot][1] > board.robot[other_robot][1]:
                        robot_actions[robot].discard('l')
                        robot_actions[other_robot].discard('r')
                    
        
        for robot in robot_actions:
            for action in robot_actions[robot].copy():
                if action=='u':
                    robot_line = board.robot_position(robot)[0]
                    robot_column = board.robot_position(robot)[1]
                    if board.down_barrier[robot_column-1][robot_line-1]:
                        robot_actions[robot].remove(action)
                elif action=='r':
                    robot_line = board.robot_position(robot)[0]
                    robot_column = board.robot_position(robot)[1]
                    if board.right_barrier[robot_line-1][robot_column]:
                        robot_actions[robot].remove(action)
                elif action=='d':
                    robot_line = board.robot_position(robot)[0]
                    robot_column = board.robot_position(robot)[1]
                    if board.down_barrier[robot_column-1][robot_line]:
                        robot_actions[robot].remove(action)
                elif action=='l':
                    robot_line = board.robot_position(robot)[0]
                    robot_column = board.robot_position(robot)[1]
                    if board.right_barrier[robot_line-1][robot_column-1]:
                        robot_actions[robot].remove(action)
        
        actions = []
        for robot in robot_actions:
            for action in robot_actions[robot]:
                actions.append((robot, action))
        
        return actions

    def result(self, state: RRState, action):
        """ Retorna o estado resultante de executar a 'action' sobre
        'state' passado como argumento. A ação retornada deve ser uma
        das presentes na lista obtida pela execução de
        self.actions(state). """
        # TODO
        
        if action in self.actions(state):
            new_state = copy.deepcopy(state)
            board = new_state.board
            if action[1]=='u':
                board.robot[action[0]] = (board.robot_position(action[0])[0]-1, board.robot_position(action[0])[1])
                final_state = self.result(new_state, action)
                del new_state
                return final_state
            elif action[1]=='r':
                board.robot[action[0]] = (board.robot_position(action[0])[0], board.robot_position(action[0])[1]+1)
                final_state = self.result(new_state, action)
                del new_state
                return final_state
            elif action[1]=='d':
                board.robot[action[0]] = (board.robot_position(action[0])[0]+1, board.robot_position(action[0])[1])
                final_state = self.result(new_state, action)
                del new_state
                return final_state
            elif action[1]=='l':
                board.robot[action[0]] = (board.robot_position(action[0])[0], board.robot_position(action[0])[1]-1)
                final_state = self.result(new_state, action)
                del new_state
                return final_state
        else:
            return state

    def goal_test(self, state: RRState):
        """ Retorna True se e só se o estado passado como argumento é
        um estado objetivo. Deve verificar se o alvo e o robô da
        mesma cor ocupam a mesma célula no tabuleiro. """
        # TODO
        board = state.board
    
        if board.robot_position(board.goal[0])[0] == board.goal[1] and board.robot_position(board.goal[0])[1] == board.goal[2]:
            return True
        else:
            return False

    def h(self, node: Node):
        """ Função heuristica utilizada para a procura A*. """
        # TODO
        
        
        board = node.state.board
        distance = abs(board.robot[board.goal[0]][0]-board.goal[1]) + abs(board.robot[board.goal[0]][1]-board.goal[2])
        return distance
            


if __name__ == "__main__":

    # TODO:
    # Ler o ficheiro de input de sys.argv[1],
    board = parse_instance(sys.argv[1])
    problem = RicochetRobots(board)
    
    # Usar uma técnica de procura para resolver a instância,
    #solution_node = breadth_first_tree_search(problem)
    #solution_node = depth_first_tree_search(problem)
    #solution_node = greedy_search(problem)
    solution_node = astar_search(problem)
    problem.h(solution_node)
    # Retirar a solução a partir do nó resultante,
    solution = []
    node = solution_node
    while(node!=None and node.action!=None):
        solution.insert(0, node.action)
        node = node.parent

    # Imprimir para o standard output no formato indicado.
    print(len(solution))
    for action in solution:
        print(action[0], action[1])