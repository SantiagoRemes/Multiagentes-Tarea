from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np

import networkx as nx 

import math as math


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Recarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.ocupado = False

class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.needs_charge = False
        self.celda_sucia_cercana = None
        self.in_charge = False

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        if len(lista_de_vecinos) == 0:
            self.sig_pos = self.pos
        else:
            self.sig_pos = self.random.choice(lista_de_vecinos).pos        

    @staticmethod
    def buscar_celdas_sucia(self):
        celdas_sucias = [obj for obj in self.model.grid.get_cell_list_contents(self.pos) if isinstance(obj, Celda) and obj.sucia]

        if len(celdas_sucias) == 0:
            for cell in self.model.grid.coord_iter():
                cell_content, pos = cell
                for obj in cell_content:
                    if isinstance(obj, Celda) and obj.sucia:
                        celdas_sucias.append(obj)

        if len(celdas_sucias) > 0:
            closest_cell = min(celdas_sucias, key=lambda c: self.distance(self.pos, c.pos))
            return [closest_cell]
        else:
            return []

    def buscar_cargadores(self):
        cargadores_no_ocupados = []
        if(self.in_charge):
            for cargador in self.model.lista_cargadores:
                cargadores_no_ocupados.append(cargador)
        else:
            for cargador in self.model.lista_cargadores:
                if not(cargador.ocupado):
                    cargadores_no_ocupados.append(cargador)
    
        if len(cargadores_no_ocupados) > 0:
            closest_cargador = min(cargadores_no_ocupados, key=lambda c: self.distance(self.pos, c.pos))
            
            return [closest_cargador]
        else:
            return []

    def distance(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return abs(x1 - x2) + abs(y1 - y2)
    
        

    def encontrar_camino(self, target):
        sig_pos_robots = [robot.sig_pos for robot in self.model.lista_robots if robot != self and robot.sig_pos is not None]

        graph = nx.Graph()
        for cell in self.model.grid.coord_iter():
            cell_content, pos = cell
            x, y = pos

            # Check if the cell is occupied by a mueble or another robot's next position
            if not any(isinstance(obj, (Mueble, RobotLimpieza)) or pos in sig_pos_robots for obj in cell_content):
                neighbors = self.model.grid.get_neighborhood(
                    pos,
                    moore=True,
                    include_center=True
                )

                for neighbor in neighbors:
                    content = self.model.grid.get_cell_list_contents(neighbor)
                    
                    # Check if the neighbor cell is free of muebles and other robots' next positions
                    if all(not(isinstance(obj, (Mueble)) or obj.pos in sig_pos_robots) for obj in content):
                        graph.add_edge(pos, neighbor)

        current_pos = self.pos
        target_pos = target.pos

        if current_pos not in graph.nodes or target_pos not in graph.nodes:
            # Handle the case where either the source or target node is not in the graph
            return []

        try:
            path = nx.shortest_path(graph, current_pos, target_pos)
            final_path = [p for p in path if all(not isinstance(obj, (Mueble)) for obj in self.model.grid.get_cell_list_contents(p))]
            return final_path
        except nx.NetworkXNoPath:
            # Handle the case where no path exists between the source and target nodes
            return []

    def step(self):
        if self.carga == 0:
            self.sig_pos = self.pos

        elif self.needs_charge:
            cargador = self.buscar_cargadores()

            if len(cargador) == 0:
                self.sig_pos = self.pos
                return
            
            if(cargador[0].pos == self.pos and self.carga != 100):
                cargador[0].ocupado = True
                self.in_charge = True
                self.sig_pos = self.pos
                self.carga = self.carga + 25
                if(self.carga >= 100):
                    self.carga = 100
                    self.needs_charge = False
                    cargador[0].ocupado = False
                    self.in_charge = False

            else:
                path_to_target = self.encontrar_camino(cargador[0])

                if len(path_to_target) > 1:
                    self.sig_pos = path_to_target[1]
                else:
                    self.sig_pos = self.pos

        elif self.carga < 30:
            self.needs_charge = True
            self.sig_pos = self.pos

        else:

            if self.celda_sucia_cercana != None:
                if self.celda_sucia_cercana.pos == self.pos:
                    self.celda_sucia_cercana.sucia = False
                    self.celda_sucia_cercana = None


            celdas_sucias = self.buscar_celdas_sucia(self)
            if len(celdas_sucias) == 0:
                self.sig_pos = self.pos
                return
            else:
                self.celda_sucia_cercana = celdas_sucias[0]
            

        
            if self.celda_sucia_cercana != None:
                target_cell = self.celda_sucia_cercana
                path_to_target = self.encontrar_camino(target_cell)
                if len(path_to_target) > 1:
                    self.sig_pos = path_to_target[1]
                else:
                    self.sig_pos = self.pos
        
                
    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1


        if self.carga > 0:
            self.model.grid.move_agent(self, self.sig_pos)
            self.carga -= 1

        self.sig_pos = None


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles
        self.lista_robots = []
        self.lista_cargadores = []

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        posiciones_cargadores = [(0,0), (19,0), (0,19), (19,19)]
        for id, pos in enumerate(posiciones_cargadores):
            bateria = Recarga(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(bateria, pos)
            posiciones_disponibles.remove(pos)
            self.lista_cargadores.append(bateria)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)
        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.lista_robots.append(robot)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0