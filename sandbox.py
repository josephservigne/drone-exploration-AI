import pybullet as p
import numpy as np
# from drone import get_resolution

SIZE = 6 # mètres
resolution = 1/30


def create_wall(position, half_extents):
    """
    Crée un 'mur' (une simple box statique) dans la scène PyBullet.
    - position: [x, y, z]
    - half_extents: [hx, hy, hz] demi-dimensions de la boîte
    """
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX, halfExtents=half_extents
    )
    body_id = p.createMultiBody(
        baseMass=0,  # masse=0 -> objet statique
        baseCollisionShapeIndex=collision_shape_id,
        basePosition=position,
    )
    return body_id 

# def create_block(position, size=1):
#     """
#     Crée un bloc de taille (size, size, size) à une position donnée.
#     - position: [x, y, z] - centre du bloc
#     - size: taille du bloc (1 par défaut)
#     """
#     half_extents = [size / 2] * 3  # Demi-tailles pour PyBullet
#     collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
#     body_id = p.createMultiBody(
#         baseMass=0,  # Objet statique
#         baseCollisionShapeIndex=collision_shape_id,
#         basePosition=position
#     )
#     return body_id


def generate_random_blocks(nb_blocks, exclusion_radius,r=0.05):
    used_positions = set()
    create_wall(position=[SIZE//2,0, 1], half_extents=[r, SIZE//2, 1])
    create_wall(position=[-SIZE//2,0, 1], half_extents=[r, SIZE//2, 1])
    create_wall(position=[0, SIZE//2, 1], half_extents=[SIZE//2, r, 1])  
    create_wall(position=[0, -SIZE//2, 1], half_extents=[SIZE//2, r, 1]) 
    create_block([1, 0, 1/2])
    while len(used_positions) < nb_blocks:
        x = np.random.randint(-SIZE//2, SIZE//2)
        y = np.random.randint(-SIZE//2, SIZE//2)

        # Vérifie la distance au centre (0,0)
        distance = np.sqrt((x - SIZE//2)**2 + (y - SIZE//2)**2)
        if distance < exclusion_radius:
            continue  # on saute ce point s'il est trop proche du centre

        if (x, y) not in used_positions:
            used_positions.add((x, y))
            create_block([x, y, 1/2])

    return int(SIZE/resolution) # 1 m = 100 pixels

def create_block(position):
    """
    Crée un bloc de dimensions 1m (largeur) x 1m (longueur) x 2m (hauteur) à une position donnée.
    - position: [x, y, z] - centre du bloc
    """
    half_extents = [0.2, 0.2, 1]  # Demi-tailles pour PyBullet
    collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    body_id = p.createMultiBody(
        baseMass=0,  # Objet statique
        baseCollisionShapeIndex=collision_shape_id,
        basePosition=position
    )
    return body_id

def create_map(r=0.05):
    create_wall(position=[SIZE//2,0, 1], half_extents=[r, SIZE//2, 1])
    create_wall(position=[-SIZE//2,0, 1], half_extents=[r, SIZE//2, 1])
    create_wall(position=[0, SIZE//2, 1], half_extents=[SIZE//2, r, 1])  
    create_wall(position=[0, -SIZE//2, 1], half_extents=[SIZE//2, r, 1]) 
    create_block([1,1,1/2])
    create_block([2,1,1/2])
    create_block([3,1,1/2])
    # create_block([1,0,1/2])
    # create_block([2.5,0,1/2])

    return int(SIZE/resolution) # 1 m = 100 pixels

def create_drone(position):
    """
    Crée un 'drone' simplifié : une simple boîte dynamique.
    Pour un vrai drone, il faudrait un fichier URDF plus réaliste
    ou un modèle multi-rotors (hélices, etc.).
    """
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[0.1, 0.1, 0.05],  # drone cubique ~ 20x20x10 cm
    )
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[0.1, 0.1, 0.05],
        rgbaColor=[0.2, 0.6, 1.0, 1.0],  # Couleur (bleu clair)
    )
    body_id = p.createMultiBody(
        baseMass=1.0,  # on suppose un drone de masse 1 kg
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
    )
    taille_x = 0.2  # 2 * 0.1
    return body_id,taille_x

def import_urdf_drone(filepath, position):
    """
    Importe un modèle URDF existant et le place à une position donnée.
    - filepath: chemin du fichier URDF
    - position: position de départ [x, y, z]
    """
    try:
        body_id = p.loadURDF(filepath, basePosition=position)
        print(f"Drone URDF importé depuis {filepath}")
        return body_id
    except Exception as e:
        print(f"Erreur lors de l'importation du URDF : {e}")
        return None


def create_red_point(position): 
    """
    Crée un point rouge à la position donnée.
    """
    sphere_radius = 0.05  # Taille du "point" (~1 cm)
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=sphere_radius,
        rgbaColor=[1.0, 0.0, 0.0, 1.0],  # Couleur rouge
    )
    body_id = p.createMultiBody(
        baseMass=0,  # Pas de masse (objet statique)
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
    )
    return body_id