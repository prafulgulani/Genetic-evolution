import pybullet as p
import csv
import numpy as np
from population import Population
from simulation import Simulation
from genome import Genome
import creature
import gc


def run_experiment(gene_count, num_generations, pop_size=30):
    sim = Simulation()
    pop = Population(pop_size=pop_size, gene_count=gene_count)

    # Setup CSV logging
    filename = f"results_gene_count_{gene_count}.csv"
    with open(filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Generation", "Best_Fitness", "Average_Fitness", "Worst_Fitness", "Mean_Links", "Max_Links"])

        best_creature = None
        best_fitness = -np.inf

        for generation in range(num_generations):

            if generation % 500 == 0:
                gc.collect()  # Force garbage collection

            print(f"Gene Count {gene_count} | Generation {generation}")

            # Initialize tracking variables
            fitness_list = np.zeros(len(pop.creatures))  
            # link_data = np.zeros(len(pop.creatures))
            # sum_fitness = 0
            # sum_links = 0
            # worst_fitness = np.inf
            best_index = -1

            # Run simulation and compute fitness in a **single loop**
            for i, cr in enumerate(pop.creatures):
                sim.run_creature(cr, iterations=2400)
                fitness_list[i] = cr.calculate_fitness()
                # link_data[i] = len(cr.get_expanded_links())

                # Store fitness and links
                # fitness_list.append(fitness)
                # link_data.append(links)
                
                # Update sum calculations
                # sum_fitness += fitness
                # sum_links += links
                
                # Track best and worst fitness in the same loop
                if fitness_list[i] > best_fitness:
                    best_fitness = fitness_list[i]
                    best_creature = cr
                    best_index = i
                # worst_fitness = np.min(worst_fitness, fitness)

            # Identify the best creature of the current generation
            if generation % 200 == 0:  
                # avg_fitness = sum_fitness / len(fitness_list)
                # mean_links = np.mean(link_data)
                # max_links = np.max(link_data)
                csv_writer.writerow([generation, best_fitness])
                print(generation, best_fitness)

            # Generate new population
            fit_map = Population.get_fitness_map(fitness_list)
            new_creatures = [None] * len(pop.creatures)
            
            # Carry over the best creature (elitism)
            new_creatures[0] = pop.creatures[best_index]
            
            for i in range(1, len(pop.creatures)):
                parent1 = pop.creatures[Population.select_parent(fit_map)]
                parent2 = pop.creatures[Population.select_parent(fit_map)]
                dna = Genome.crossover(parent1.dna, parent2.dna)
                dna = Genome.point_mutate(dna, rate=0.1, amount=0.2)
                dna = Genome.shrink_mutate(dna, rate=0.20)
                dna = Genome.grow_mutate(dna, rate=0.1)

                cr = creature.Creature(gene_count)
                cr.update_dna(dna)
                new_creatures[i] = cr
            
            pop.creatures = new_creatures
            
    print(f"Experiment with gene count {gene_count} completed. Results saved to {filename}.")
    # Save elite DNA
    if best_creature:
            best_filename = f"best_gene_count_{gene_count}.csv"
            Genome.to_csv(best_creature.dna, best_filename)
            print(f"Best creature for gene count {gene_count} saved to {best_filename}.")


if __name__ == "__main__":
    # Run experiments for different gene counts
    # for gene_count in range(4, 9, 1):
    #     run_experiment(gene_count=gene_count, num_generations=3001)
    run_experiment(gene_count=4, num_generations=3001)
    