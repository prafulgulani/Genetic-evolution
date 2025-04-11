import genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np

class MotorType(Enum):
    PULSE = 1
    SINE = 2

class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        if control_waveform <= 0.5:
            self.motor_type = MotorType.PULSE
        else:
            self.motor_type = MotorType.SINE
        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
            
        if self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)
        
        return output 


class Creature:
    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

    def get_flat_links(self):
        if self.flat_links == None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links
    
    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], 
                                self.flat_links[0].name, 
                                self.flat_links, 
                                exp_links)
        self.exp_links = exp_links

        # Log the total number of expanded links
        # print(f"Expanded Links: {len(exp_links)}")
    
        # Limit the total number of links
        max_links = 127
        if len(exp_links) > max_links:
            exp_links = exp_links[:max_links]
            print(f"Link count capped at {max_links}")
        self.exp_links = exp_links
        return self.exp_links

    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:# skip the root node! 
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe") #  choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_expanded_links()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors 
    
    def update_position(self, pos):
        if self.start_position == None:
            self.start_position = pos
        else:
            self.last_position = pos

    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1-p2)
        return dist 

    def calculate_fitness(self):
        """Compute fitness with power-based distance rewards and NumPy optimizations."""
        if self.start_position is None or self.last_position is None:
            return 0

        # Convert positions to NumPy arrays
        start_pos = np.array(self.start_position)
        last_pos = np.array(self.last_position)

        # Calculate movement components
        xy_travel = np.linalg.norm(last_pos[:2] - start_pos[:2])  # Distance in XY plane
        height_gain = max(0, last_pos[2] - start_pos[2])  # Height climbed

        # Apply power function for progressive reward
        distance_reward = xy_travel**2  # Squared reward to strongly favor larger distances
        height_reward = height_gain**2  

        # Penalize unnatural movement (too much height vs. distance)
        ratio = height_gain / max(0.1, xy_travel)
        movement_penalty = 0.5 if ratio > 1.2 else 1.0

        # Reward for moving toward mountain center
        dist_to_center = np.linalg.norm(last_pos[:2])  # Distance from (0,0)
        center_reward = 1.0 / (dist_to_center + 1)

        # Link efficiency penalty
        link_count = len(self.get_expanded_links())
        link_penalty = 0.3 * max(0, link_count - 80)
        link_reward = 0.1 * link_count

        # Compute final fitness
        fitness = (distance_reward + height_reward) * movement_penalty + center_reward + link_reward - link_penalty
        return max(0, fitness)  # Ensure fitness is non-negative

    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None