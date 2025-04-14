# Genome Simulation and Evolution

This project simulates the evolution of creatures with genomes that encode physical traits and motor control. The project is inspired by Karl Sims' work on artificial life and creature evolution, which explores how simple genetic representations can give rise to complex behaviors.

The goal of this project is to simulate a creature which can climb the mountain, which is made using PyBullet for physics simulation and a Gaussian function for terrain generation. 

The creatures evolve over successive generations and learn to climb the mountain more efficiently. Each generation undergoes selection, crossover, and mutation to improve performance. The best-performing creature, based on a fitness function, is passed on to the next generation this is called elitism. The fitness score is calculated using multiple parameters such as distance traveled, height climbed, proximity to the mountain center, and the number of physical links in the creature. Based on these scores, parent genomes are selected, combined through crossover, and slightly altered through mutation to generate a new population, promoting gradual improvement in climbing ability.
