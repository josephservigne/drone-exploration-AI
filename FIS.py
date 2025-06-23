import numpy as np

resolution = 1/30

class FIS:

    """
        Représente une Frontier Information Structure  (FIS) dans une carte voxel 2D.

        Une FIS correspond à un regroupement de cellules situées en bordure de l'espace exploré,
        pouvant mener à des zones inexplorées. Chaque FIS est caractérisée par :
        - un identifiant unique (cluster_id),
        - un ensemble de cellules (voxels) qui la composent,
        - une position moyenne (pour orienter le drone),
        - une boîte englobante (AABB) pour des tests spatiaux rapides,
        - une liste de points de vue d'exploration autour de la FIS.

    """

    def __init__(self, cluster_id,cells):
        self.cluster_id = cluster_id
        self.cells = cells  # Liste des voxels de la frontière
        self.average_position = None
        self.aabb = None  # Axis-Aligned Bounding Box
        self.viewpoints = []  # Points de vue pour exploration

    def compute_average_position(self):
        """Calcule la position moyenne des voxels de la frontière."""
        if not self.cells:
            return None
        self.average_position = np.mean(self.cells, axis=0)

    def compute_aabb(self):
        """Calcule l'Axis-Aligned Bounding Box (AABB) de la frontière."""
        if not self.cells:
            return None
        min_coords = np.min(self.cells, axis=0)
        max_coords = np.max(self.cells, axis=0)
        self.aabb = (min_coords, max_coords)

def compute_Bm_from_updated_cells(updated_cells):

    """
        Calcule la plus petite boîte englobante (AABB) 2D autour d'un ensemble de cellules mises à jour.

        Cette boîte englobante est définie par ses coins supérieur-gauche (min_i, min_j)
        et inférieur-droit (max_i, max_j).

        Parameters
        ----------
        updated_cells : list of tuple
            Liste de cellules (i, j) mises à jour dans la grille.

        Returns
        -------
        tuple or None
            Un tuple ((min_i, min_j), (max_i, max_j)) représentant l’AABB minimale englobant les points,
            ou None si la liste est vide.
    """
        
    if not updated_cells:
        return None
    
    arr = np.array(updated_cells)

    # coordonnées minimales et maximales des points
    min_coords = arr.min(axis=0)
    max_coords = arr.max(axis=0)

    return (tuple(min_coords), tuple(max_coords))  # ((min_i, min_j), (max_i, max_j))

def aabb_intersect(aabb1, aabb2):

    """
        Vérifie si deux boîtes englobantes (AABB) s'intersectent dans une grille 2D.

        Deux AABB s'intersectent si elles partagent au moins une cellule commune
        (chevauchement horizontal et vertical).

        Parameters
        ----------
        aabb1 : tuple
            Première boîte englobante sous la forme ((min_i, min_j), (max_i, max_j)).
        aabb2 : tuple
            Deuxième boîte englobante sous la même forme.

        Returns
        -------
        bool
            True si les deux AABB se chevauchent, False sinon.
    """
        
    (min1_i, min1_j), (max1_i, max1_j) = aabb1
    (min2_i, min2_j), (max2_i, max2_j) = aabb2

    # condition de non-chevauchement sur les deux axes
    return not (max1_i < min2_i or max2_i < min1_i or
                max1_j < min2_j or max2_j < min1_j)





    
    



