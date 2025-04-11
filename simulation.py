import pybullet as p
from multiprocessing import Pool
import cw_envt as envt
import pybullet_data as pd
import numpy as np


class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id

    def run_creature(self, cr, iterations=2400):
        pid = self.physicsClientId

        # Reset simulation and initialize environment
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
        p.setGravity(0, 0, -10, physicsClientId=pid)

        # Load the mountain environment
        p.setAdditionalSearchPath(pd.getDataPath())
        p.setAdditionalSearchPath('shapes/')
        mountain_position = (0, 0, -1)
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
        p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1, physicsClientId=pid)
        envt.make_arena(arena_size=40, wall_height=3)

        # Save and load creature URDF
        xml_file = f'temp{self.sim_id}.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)
        
        try:
            cid = p.loadURDF(xml_file, physicsClientId=pid)
            if cid < 0:
                raise RuntimeError(f"Invalid creature ID: {cid}")
        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {xml_file}") 
        
        start_position = [5, 5, 2.5]
        p.resetBasePositionAndOrientation(cid, start_position, [0, 0, 0, 1], physicsClientId=pid)

        # Run simulation
        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            if step % 24 == 0:
                self.update_motors(cid=cid, cr=cr)

            pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)        
    
    def update_motors(self, cid, cr):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        for jid in range(p.getNumJoints(cid,
                                        physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            p.setJointMotorControl2(cid, jid, 
                    controlMode=p.VELOCITY_CONTROL, 
                    targetVelocity=m.get_output(), 
                    force = 5, 
                    physicsClientId=self.physicsClientId)
            
    def eval_population(self, pop, iterations=2400):
        fitness_list = []
        for cr in pop.creatures:
            fitness = self.run_creature(cr, iterations)
            fitness_list.append(fitness)
        return fitness_list
        

    # You can add this to the Simulation class:
    # def eval_population(self, pop, iterations):
    #     for cr in pop.creatures:
    #         self.run_creature(cr, 2400) 


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures