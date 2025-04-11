import creature 
import numpy as np

class Population:
    def __init__(self, pop_size, gene_count):
        self.creatures = [creature.Creature(
                          gene_count=gene_count) 
                          for _ in range(pop_size)]

    @staticmethod
    def get_fitness_map(fits):
        return np.cumsum(fits)  # Fast cumulative sum
    
    @staticmethod
    def select_parent(fitmap):
        r = np.random.rand() * fitmap[-1]  # Random number within total fitness
        return np.searchsorted(fitmap, r)  # Fast binary search

