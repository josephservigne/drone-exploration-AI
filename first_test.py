import cv2
import pybullet as p
import pybullet_data


from drone import *
from sandbox import *

def load_data():

    """
        Chargement des données de pybullet, du drone et de la map
    """

    # Configuration de la simulation
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setRealTimeSimulation(0) 
    p.setGravity(0, 0, 0) # mettre la gravité à -9.81 en dernier paramètre
    p.loadURDF("plane.urdf") # sol plat

    # Chargement des données
    size_map = generate_random_blocks(10,1)
    drone = Drone([0, 0, 0.5],size_map,size_map)

    return drone


def main():

    drone=load_data()
    start = True # nécéssaire pour lancer la boucle à l'initialisation 
    
    while drone.list_FIS or start:

        if start: start=False

        # mise à jour du drone, de la map et affichage de la reconstitution de la map
        image = drone.update_voxel_grid_map()
        cv2.imshow("Map", image)
        p.stepSimulation()
        
        # on parcourt la FIS la plus proche
        drone.reach_FIS()
        

if __name__ == "__main__":
    main()

