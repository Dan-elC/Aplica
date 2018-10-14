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

def invertir(lista):
    nueva_lista = []
    for i in range(len(lista)):
        if lista[i] =='down':
            nueva_lista.append('up')
        if lista[i] =='up':
            nueva_lista.append('down')
        if lista[i] =='right':
            nueva_lista.append('left')
        if lista[i] =='left':
            nueva_lista.append('right')
    return nueva_lista


def inFrontera(e,frontera):
    
    for i in range(len(frontera.list)):
        (estado, camino, costo)= frontera.list[i]
        if e==estado:
            return i+1
    return 0


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def graph_search(problem, frontera):

    "Se inserta a modo de nodo el estado, camino y costo = (estado,camino,costo)"
    estadoInicial = problem.getStartState()
    frontera.push((estadoInicial,[],0))
    explorados = set() #Se coloca como set() y no como list(), ya que los valores no se repiten

    #Siguiendo los pasos similares al lab 1
    while(frontera):
        #node = frontier.pop()
        (estado,camino,costo) = frontera.pop()
        if(problem.isGoalState(estado)):
                        
            
            print ('\n******************************************************************')
            print ('*                                                                *')
            print ('*       Numero de nodos visitados: {}                            *').format(len(explorados))
            print ('*       Numero de nodos almacenados en memoria: {}               *').format(len(frontera.list))
            print ('*       Costo de la solucion:  %d                                *' % costo)
            print ('*                                                                *')
            print ('******************************************************************')

            print ('\nSe encontro un camino de %d movimientos: %s \n' % (costo, str(camino)))


            return camino
        explorados.add(estado)
        sucesores = problem.getSuccessors(estado)
        #Obtener sucesores
        for sucesor in sucesores:
            #Si sucesor no se encuentra en explorados ni en la frontera
            if sucesor[0] not in explorados and (inFrontera(sucesor[0],frontera)==0):
                frontera.push((sucesor[0],camino+[sucesor[1]], costo + sucesor [2]))


def depthFirstSearch(problem):

    frontier = util.Stack()
    return graph_search(problem,frontier)


def breadthFirstSearch(problem):

    frontier = util.Queue()
    return graph_search(problem,frontier)
    

def recursiveDLS(nodo, problem, limite,visitados):
    (estado, camino, costo) = nodo
    visitados.add(estado)
    #print estado[0] , estado[1] , limite
    if problem.isGoalState(estado):

        print ('\n******************************************************************')
        print ('*                                                                *')
        print ('*       Numero de nodos visitados: {}                            *').format(len(visitados))
        print ('*       No existe nodos en frontera                              *')
        print ('*       Costo de la solucion:  %d                                *' % costo)
        print ('*                                                                *')
        print ('******************************************************************')


        print ('\nSe encontro un camino de %d movimientos: %s \n' % (costo, str(camino)))

        return camino
    
    elif limite == 0:
        return 'cutoff'
    else:
        interrumpio = False
        sucesores=problem.getSuccessors(estado)
        for sucesor in sucesores:
            hijo = (sucesor[0], camino + [sucesor[1]], costo + sucesor[2])
            resultado = recursiveDLS(hijo, problem, limite-1,visitados)
            if resultado == 'cutoff':
                interrumpio = True
            elif(resultado != 'failure'):
                return resultado
        if interrumpio:
            return 'cutoff'
        else:
            return 'failure'

def depthLimiteedSearch(problem, limite):
    visitados = set()
    return recursiveDLS((problem.getStartState(),[],0), problem, limite,visitados)
    

def iDeepeningSearch(problem):
    "***cutoff=[0]  |   failure=[-1]***"
    "***cutoff='cutoff'  |   failure='failure'***"

    limite = 0

    #Se toma un while, ya que no existe un for hasta el infinito
    while True:
        resultado = depthLimiteedSearch(problem, limite)
        if resultado != 'cutoff':
            #print('Se ha encontrado un camino de %d movimientos: %s' % (len(resultado), str(resultado)))
            return resultado
        limite +=1


def BidirectionalSearch(problem):

    "***ESTADO INICIAL***"
    fronteraIni = util.Queue()
    estadoInicial= problem.getStartState()
    fronteraIni.push((estadoInicial, [],0))
    visitadosIni=set()
    visitadosIni.add(estadoInicial)

    "***ESTADO OBJETIVO***"
    fronteraObj=util.Queue()
    estadoObjetivo = problem.goal
    #estadoObjetivo= (problem.startingPosition,[True,True,True,True,False])
    fronteraObj.push((estadoObjetivo, [],0))
    visitadosObj=set()
    visitadosObj.add(estadoObjetivo)

    while not(fronteraIni.isEmpty()) and not(fronteraObj.isEmpty()):
    	(estado1, camino1, costo) =fronteraIni.pop()
    	if inFrontera(estado1,fronteraObj):
    		(a,camino2,b) = fronteraObj.list[inFrontera(estado1,fronteraObj)-1]
    		break

    	sucesores=problem.getSuccessors(estado1)
    	for sucesor in sucesores:
    		if sucesor[0] not in visitadosIni:
    			fronteraIni.push((sucesor[0], camino1 + [sucesor[1]], costo + sucesor[2]))
    			visitadosIni.add(sucesor[0])

    	(estado, camino2, costo) =fronteraObj.pop()
    	if inFrontera(estado,fronteraIni):
    		(a,camino1,b) = fronteraIni.list[inFrontera(estado,fronteraIni)-1]
    		break

    	sucesores=problem.getSuccessors(estado)
    	for sucesor in sucesores:
    		if sucesor[0] not in visitadosObj:
    			fronteraObj.push((sucesor[0], camino2 + [sucesor[1]], costo + sucesor[2]))
    			visitadosObj.add(sucesor[0])

    camino2.reverse()
    #camino2_inv = camino2
    camino2_inv= invertir(camino2)


    print ('\n******************************************************************')
    print ('*                                                                *')
    print ('*       Numero de nodos visitados: {}                            *').format(len(visitadosIni)+len(visitadosObj))
    print ('*       Numero de nodos almacenados en memoria: {}               *').format(len(fronteraObj.list)+len(fronteraIni.list))
    print ('*       Costo de la solucion:  %d                                *' % costo)
    print ('*                                                                *')
    print ('******************************************************************')

    print ('\nSe encontro un camino de %d movimientos: %s \n' % (costo, str(camino1 + camino2_inv)))



    #print ('Cantidad de nodos en memoria: {}').format(len(fronteraObj.list)+len(fronteraIni.list))
    return camino1 + camino2_inv


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattan_distance(state,problem):

    tamanio = len(state.cells)
    sum = 0
    for i in range(tamanio):
        for j in range(tamanio):
            tile = state.cells[i][j]
            for m in range(tamanio):
                for n in range(tamanio):
                    if tile ==  problem.goal.cells[m][n]:
                        sum += abs(i-m) + abs(j-n)

    return sum


def euclideanHeuristic(state, problem):
    
    tamanio = len(state.cells)
    sum = 0
    for i in range(tamanio):
        for j in range(tamanio):
            tile = state.cells[i][j]
            for m in range(tamanio):
                for n in range(tamanio):
                    if tile ==  problem.goal.cells[m][n]:
                        sum += ((i-m)**2 + (j-n)**2)

    return (sum)**0.5


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** costo de heuristica ***"
    cost = lambda node: problem.getCostOfActions([x[1] for x in node]) + heuristic(node[len(node)-1][0], problem)
    frontier = util.PriorityQueueWithFunction(cost)
    explored = set()
    "Se inserta a modo de nodo el estado, camino y costo = (estado,camino,costo)"
    frontier.push([(problem.getStartState(), [], 0)])


    while frontier:
        "***nodo[][0]=estado      |       nodo[][1]=camino      |       nodo[][2]=costo***"
        nodo = frontier.pop()
        #print(path[-1][0])
        #Extraer estado del ultimo nodo
        estado = nodo[-1][0]        

        if problem.isGoalState(estado):
            camino = [x[1] for x in nodo][1:]


            print ('\n******************************************************************')
            print ('*                                                                *')
            print ('*       Numero de nodos visitados: {}                            *').format(len(explored))
            print ('*       Numero de nodos almacenados en memoria: {}               *').format(len(frontier.heap))
            print ('*       Costo de la solucion:  %d                                *' % len(camino))
            print ('*                                                                *')
            print ('******************************************************************')

            print ('\nSe encontro un camino de %d movimientos: %s \n' % (len(camino), str(camino)))



            #print('Nodos encontrados en frontera: %d' % len(frontier.heap)) #[x[1] for x in path][1:]
            #print('A* found a path of %d moves: %s' % (len(camino), str(camino)))
            return camino

        if estado not in explored:
            explored.add(estado)
            for successor in problem.getSuccessors(estado):
                if successor[0] not in explored:
                    successorPath = nodo[:]
                    successorPath.append(successor)
                    frontier.push(successorPath)
                    if (heuristic(nodo[-1][0], problem) > problem.getCostOfActions(
                            [x[1] for x in successorPath]) + heuristic(successorPath[-1][0], problem)):
                        print(heuristic(nodo[-1][0], problem),
                              problem.getCostOfActions([x[1] for x in successorPath]) + heuristic(successorPath[-1][0], problem))

    return []
    
		
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
