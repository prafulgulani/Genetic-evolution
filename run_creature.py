import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import genome
import creature
import cw_envt as envt


def visualize_creature(filename, iterations=2400, total_time=10):
    if not filename:
        print("No filename provided. Exiting.")
        return

    print(f"Loading DNA from {filename}...")
    dna = genome.Genome.from_csv(filename)
    gene_count = len(dna)  # Infer gene count from DNA

    # Initialize the creature
    cr = creature.Creature(gene_count=gene_count)
    cr.update_dna(dna)

    # Start PyBullet environment
    p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pd.getDataPath())

    # Load the mountain environment
    p.setAdditionalSearchPath('shapes/')
    mountain_position = (0, 0, 0)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    envt.make_arena(arena_size=40, wall_height=12)

    # Save creature as URDF and load it into the simulation
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    rob1 = p.loadURDF('test.urdf', [5, 5, 10])
    start_pos, _ = p.getBasePositionAndOrientation(rob1)

    elapsed_time = 0
    wait_time = 1.0 / 240  # Seconds per simulation step
    step = 0

    while elapsed_time < total_time:
        p.stepSimulation()
        step += 1

        # Update motors periodically
        if step % 30 == 0:
            motors = cr.get_motors()
            assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
            for jid in range(p.getNumJoints(rob1)):
                mode = p.VELOCITY_CONTROL
                vel = motors[jid].get_output()
                p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)

        # Get the current position of the creature
        current_pos, _ = p.getBasePositionAndOrientation(rob1)
        print(f"Step: {step}, Current Height: {current_pos[2]}")

        time.sleep(wait_time)
        elapsed_time += wait_time

    print("Simulation Complete.")
    final_pos, _ = p.getBasePositionAndOrientation(rob1)
    print(f"Final Height Climbed: {final_pos[2]}")

    # Cleanup
    p.disconnect()


if __name__ == "__main__":
    # filename = "jump.csv"  
    # filename = "jump2.csv"  
    filename = "best_gene_count_4.csv"
    visualize_creature(filename, iterations=2400, total_time=60)
