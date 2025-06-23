import pybullet as p
import math
import numpy as np
from FIS import *
import cv2
import heapq
from sandbox import create_drone


class Drone:

    """
        Classe représentant un drone explorateur dans un environnement voxelisé 2D.

        Cette classe encapsule les informations et comportements nécessaires à 
        la navigation et à l'exploration autonome dans une carte générée à partir
        d'une caméra de profondeur. Le drone maintient une représentation locale 
        de l'environnement sous forme de grille (voxel grid), ainsi qu'une liste 
        de FIS - Frontier Information Structure - cf class FIS .

        La carte `voxel_grid_map` est une conversion discrète de l'environnement
        réel (en mètres) en une grille de pixels, via le paramètre `resolution`.

        États possibles de chaque cellule dans la grille :
        - `0` : cellule inexplorée,
        - `1` : cellule explorée et libre,
        - `2` : cellule occupée (obstacle).

        Attributs de classe
        -------------------
        width : int
            Largeur de la vue caméra en pixels (convertie depuis les mètres).
        height : int
            Hauteur de la vue caméra en pixels.
        fov : float
            Champ de vision (Field of View) de la caméra du drone, en degrés.
        max_range : float
            Portée maximale du capteur de profondeur, en mètres.

        Attributs d'instance
        --------------------
        drone_id : int
            Identifiant unique du drone dans l'environnement PyBullet.
        lenght : float
            Taille (longueur) du drone, convertie en pixels.
        speed : list[float]
            Vitesse du drone selon les axes x, y et z.
        depth_matrix : np.ndarray or None
            Matrice de profondeur capturée par la caméra du drone.
        voxel_grid_map : np.ndarray
            Représentation locale 2D de l'environnement en voxels.
        list_FIS : list[FIS]
            Liste des clusters de frontières d’intérêt détectées par le drone.
    """

    width, height = int(3/resolution), int(2.5/resolution) # 3 m * 2.5 m convertit
    fov = 60  # en degrés
    max_range = 1 # distance max auquel peut voir le drone

    voxel_grid_map : np
    list_FIS : list[FIS]

    def __init__(self,position,sizex,sizey):
        
        (self.drone_id,self.lenght) = create_drone(position)
        self.lenght = self.lenght/resolution 

        self.speed=[0.05,0.05,0] 

        self.depth_matrix = None
        self.voxel_grid_map = np.zeros((sizex, sizey), dtype=int)
        self.list_FIS=[]



# === Getters ===

    def get_id(self):
        return self.drone_id
    
    def get_position(self,d=''):

        """
            Retourne la position actuelle du drone dans l'espace 3D.

            Paramètres
            ----------
            d : str, optionnel
                Axe de la position à retourner : 'x', 'y', 'z'.
                Si aucun axe n'est spécifié (valeur par défaut ''), renvoie le tuple complet (x, y, z).

            Return
            -------
            float ou tuple
                Coordonnée demandée ou position complète du drone.
        """

        pos, _ = p.getBasePositionAndOrientation(self.drone_id)
        if d=='x': return pos[0]
        if d=='y': return pos[1]
        if d=='z': return pos[2]
        return pos
    
    def get_speed(self,d=''):

        """
            Retourne la vitesse du drone.

            Paramètres
            ----------
            d : str, optionnel
                Axe de la vitesse à retourner : 'x', 'y', 'z'.
                Si aucun axe n'est spécifié (valeur par défaut ''), renvoie le vecteur complet (vx, vy, vz).

            Return
            -------
            float ou tuple
                Composante de la vitesse selon l'axe spécifié ou vecteur complet.
        """

        if d=='x': return self.speed[0]
        if d=='y': return self.speed[1]
        if d=='z': return self.speed[2]
        return self.speed
    
    def get_depth_matrix(self):
        return self.depth_matrix

    def get_orientation(self):

        """
            Retourne l'orientation du drone (angle en radians autour de l'axe Z).

            Return
            -------
            float
                Orientation (yaw) en radians.

        """
        _, rot_quat = p.getBasePositionAndOrientation(self.drone_id)
        _, _, rot_z = p.getEulerFromQuaternion(rot_quat)
        return rot_z



# === Méthode principale ===

    def update_voxel_grid_map(self):
        """
            Méthode principale : Met à jour la carte voxel 2D à partir des données de la caméra de profondeur du drone.

            Cette méthode :
            - Calcule l'image de profondeur à l'aide du capteur du drone.
            - Parcourt les rayons dans le champ de vision (FOV) pour déterminer les points libres, occupés et inexplorés.
            - Met à jour la carte des voxels avec les données des obstacles détectés.
            - Identifie les points frontière à gauche, droite et au plus loin pour une exploration potentielle.

            Returns
            -------
            np.ndarray
                Une image mise à jour représentant la carte voxel actuelle pour affichage ou traitement.
        """

        # on calcul la matrice de profondeur du sensor
        self.compute_depth_camera_image()

        # on récupère les données du drone
        x,y = self.get_position('x'),self.get_position('y')
        rot_z = self.get_orientation()
        l1, n_rays = self.depth_matrix.shape
        minj,maxj=-(n_rays // 2)+1, n_rays // 2

        newpoint = []
        bruit = []

        FIS_list = [[],[],[]] # frontière gauche,droite et haut du sensor

        for j in range(minj,maxj): # on parcourt de gauche à droite

            angle = rot_z + j * np.deg2rad(self.fov / n_rays) # angle jusqu'auquel le drone doit regarder

            depth,max_dist = self._compute_depth_and_maxdist(l1,n_rays,j)

            n_steps = int(max_dist / resolution) # point le plus loin du drone 

            for i in range(n_steps-1,-1,-1): # on part du point max jusqu'au drone

                # on calcul le nouveau point en projetant dans la hgrid
                xi, yi = compute_new_points(x,y,i * resolution,angle,self.voxel_grid_map.shape)
                bruit.append((xi,yi))

                # on regarde si le premier point c'est à dire le plus loin du drone (i=n_steps-1) à
                # une pronfondeur inférieur à la profondeur max du sensor, on met à jour la hgrid si c'est le cas
                self._detect_obstacle(depth,xi,yi,i,n_steps)

                if isFree(self.voxel_grid_map[xi][yi]):

                    # on observe les différents cas de ce nouveau point (libre, appartenant à une FIS...)
                    self._update_free_point(xi,yi,newpoint,FIS_list,j,i,minj,maxj,n_steps)


        # on met à jour possiblement les FIS existantent et on ajoute les nouvelles
        
        if newpoint: self._update_FIS(newpoint,FIS_list,remove_bruit=False)
        
        # on regarde si du bruit peut être supprimé
        else : self._update_FIS(bruit,FIS_list,remove_bruit=True)

        return self.update_image(True)

# --- Update Functions ---

    def remove_FIS(self,fis):

        """ Enlève la fis de FIS si c'est possible """

        if fis in self.list_FIS: 
            self.list_FIS.remove(fis)
    
    def add_FIS(self,tab):

        """
            Ajoute un nouveau cluster FIS (Frontière d'Intérêt Stratégique) à la liste, 
            à condition qu'il contienne suffisamment de points pour être significatif.

            Paramètres
            ----------
            tab : list of tuple
                Liste de points représentant une frontière détectée (composante connexe).

            Retourne
            -------
            None
                Modifie `self.list_FIS` en y ajoutant un nouvel objet FIS avec ses attributs calculés,
                seulement si la frontière contient plus de 2 points (filtrage du bruit).
        """

        if len(tab)<=2: 
            return # on évite d'ajouter du bruit

        F = FIS(len(self.list_FIS),tab)

        # calcul des attributs
        F.compute_average_position()
        F.compute_aabb()

        pov = self._sample_viewpoints_circle(F.average_position)
        F.viewpoints = self._select_best_viewpoint(pov,F.average_position)

        if F.viewpoints:
            insert_idx = self._idx_optimal_FIS(F)

            self.list_FIS.insert(insert_idx, F)
    
# --- Private Helpers ---

    def _detect_obstacle(self, h, last_xi, last_yi, i, n_steps):

        """
            Marque un point comme obstacle dans la carte voxel si un obstacle a été détecté à la distance maximale du rayon.

            Cette méthode ne met à jour la carte que pour le premier point rencontré le long d’un rayon (i.e. le plus éloigné).
            Si ce point est plus proche que la portée maximale du capteur, il est considéré comme un obstacle
            et sa cellule est marquée comme occupée (valeur 2) dans la carte voxel.

            Paramètres
            ----------
            h : float
                Profondeur mesurée (distance à l’obstacle détecté, ou à la portée maximale si aucun).
            last_xi : int
                Coordonnée x du point considéré le plus loin sur le rayon.
            last_yi : int
                Coordonnée y du point considéré le plus loin sur le rayon.
            i : int
                Index courant dans le parcours du rayon (du plus loin vers le drone).
            n_steps : int
                Nombre total d'étapes (points) le long du rayon.

            Retourne
            -------
            None
                Met à jour `self.voxel_grid_map` si un obstacle est détecté.
        """

        # on regarde si le premier point (le plus éloigné) a une profondeur inférieure à la portée max du capteur
        if i != n_steps - 1:
            return

        epsilon = 0.05  # tolérance de détection (5 cm)

        if h < self.max_range - epsilon:
            self.voxel_grid_map[last_xi][last_yi] = 2  # obstacle

    def _compute_depth_and_maxdist(self,l1,n_rays,j):

        """
            Calcule la profondeur à un certain rayon et la distance maximale de détection à partir de la matrice de profondeur.

            Paramètres
            ----------
            l1 : int
                Hauteur de la matrice de profondeur (nombre de lignes)
            n_rays : int
                Nombre total de rayons projetés (largeur de la matrice)
            j : int
                Index du rayon actuellement traité (par rapport au centre)

            Returns
            -------
            tuple[float, float]
                - `depth` : profondeur mesurée par le capteur pour le rayon j 
                - `max_dist` : distance maximale de détection
        """

        depth = self.depth_matrix[l1//2][n_rays // 2 - j]

        if not np.isnan(depth): max_dist = min(depth, self.max_range) # depth peut être égal à Nan à cause du bruit
        else: max_dist= self.max_range

        return depth,max_dist

    def _update_free_point(self,xi,yi,newpoint,FIS_list,j,i,minj,maxj,n_steps):

        """
            Met à jour les points libres et les frontières dans la carte voxel.

            Cette méthode vérifie si le point (xi, yi) est inexploré,
            l'ajoute à la liste des nouveaux points explorés, et met à jour la carte voxel.
            Ensuite, elle identifie si ce point appartient à l'une des frontières spécifiques :
            la frontière la plus à gauche, la plus à droite, ou la plus éloignée.

            Paramètres
            ----------
            xi : int
                Coordonnée x du point à analyser.
            yi : int
                Coordonnée y du point à analyser.
            newpoint : list
                Liste des nouveaux points libres détectés, mise à jour par la fonction.
            FIS_list : list of lists
                Liste contenant trois sous-listes représentant les frontières gauche, droite, et la plus éloignée.
            j : int
                Index courant dans la boucle parcourant les rayons horizontaux (de gauche à droite).
            i : int
                Index courant dans la boucle parcourant les pas le long d’un rayon (du plus loin au plus proche).
            minj : int
                Index minimum des rayons horizontaux (borne gauche).
            maxj : int
                Index maximum des rayons horizontaux (borne droite).
            n_steps : int
                Nombre total de pas le long d’un rayon.

            Returns
            -------
            None
                Modifie `newpoint`, `FIS_list` et `voxel_grid_map` en place.
        """
            
        # on regarde si le point est un point inexploré 
        if self.voxel_grid_map[xi][yi]==0:
            newpoint.append((xi,yi))
            self.voxel_grid_map[xi][yi] = 1 

        # test pour savoir si c'est un nouveau point de la frontière la plus à gauche
        if j==minj and (xi,yi) in newpoint:
            FIS_list[0].append((xi,yi))

        # droite
        elif j+1==maxj and (xi,yi) in newpoint: 
            FIS_list[1].append((xi,yi))

        # le plus loins
        elif i==n_steps-1 and (xi,yi) in newpoint:
            FIS_list[2].append((xi,yi))

    def _update_FIS(self,newpoint,FIS_list,remove_bruit):

        """
            Met à jour la liste des FIS après l'exploration de nouveaux points.

            Cette méthode :
            - Calcule une boîte englobante (Bm) autour des nouveaux points explorés.
            - Supprime les clusters FIS existants (dans `self.list_FIS`) dont la boîte englobante intersecte Bm.
            - À partir des nouvelles frontières (gauche, droite, éloignée) collectées dans `FIS_list`, elle reconstruit de nouveaux clusters FIS à l'aide d'une détection de composantes connexes, et les ajoute à la liste des FIS.
            - Si remove_bruit=True elle peut supprimer le cas où des FIS seraient créer à cause du bruit

            Paramètres
            ----------
            newpoint : list of tuple
                Liste des nouveaux points libres détectés lors de la mise à jour de la carte voxel.
            FIS_list : list of lists
                Liste contenant trois sous-listes correspondant aux points appartenant aux frontières :
                - FIS_list[0] : frontière gauche
                - FIS_list[1] : frontière droite
                - FIS_list[2] : frontière la plus éloignée

            remove_bruit : bool
                Permet de gérer le cas où possiblement il peut avoir du bruit (des FIS à enlever)

            Retourne
            -------
            None
                Met à jour `self.list_FIS` en supprimant et ajoutant des clusters FIS selon les nouvelles données.
        """

        Bm = compute_Bm_from_updated_cells(newpoint) # boîte englobante autour des nouveaux points explorés

        if Bm: 
            # on regarde les FIS déjà présente s'ils elles intersectent notre bôite
            Bi_list = [F for F in self.list_FIS if F.aabb and aabb_intersect(F.aabb, Bm)]

            # si c'est le cas on les enlève
            for cluster in Bi_list:
                if not remove_bruit: self.remove_FIS(cluster)
                else: self._remove_noise(cluster)


            
            for front in FIS_list:
                clusters = find_clusters(front,self.voxel_grid_map.shape)
                for FIS in clusters:
                    self.add_FIS(FIS)

    def _remove_noise(self,cluster):
        x,y = cluster.average_position
        n1,n2 = self.voxel_grid_map.shape
        xi,yi = int(x),int(y)
        if (xi==0 or xi==n1-1 or yi==0 or yi==n2-1): self.remove_FIS(cluster)
        else: 

            # entouré d'exploré ou d'inexploré
            if self.voxel_grid_map[xi+1][yi]==1 and self.voxel_grid_map[xi][yi+1]==1 \
            and self.voxel_grid_map[xi-1][yi]==1 and self.voxel_grid_map[xi][yi-1]==1 \
            or self.voxel_grid_map[xi+1][yi]==2 and self.voxel_grid_map[xi][yi+1]==2 \
            and self.voxel_grid_map[xi][yi-1]==2 and self.voxel_grid_map[xi-1][yi]==2:
                self.remove_FIS(cluster)
        

    def _sample_viewpoints_circle(self,center, radius=24, num_samples=20):
        """
            Échantillonne aléatoirement dans la grid des positions valides autour d'un point central dans un rayon donné.

            Cette méthode génère `num_samples` points aléatoires uniformément répartis dans un cercle
            de rayon `radius` autour du point `center`. Elle retourne uniquement les points qui sont à
            l'intérieur de la grille et situés dans des cellules libres (non occupées dans la carte voxel).

            Paramètres
            ----------
            center : tuple
                Coordonnées (x, y) du centre du cercle autour duquel les viewpoints sont échantillonnés.
            radius : float, optionnel
                Rayon maximal du cercle d’échantillonnage. Par défaut à 24.
            num_samples : int, optionnel
                Nombre de points à tenter d’échantillonner. Par défaut à 20.

            Retourne
            -------
            list of tuple
                Liste de tuples (x, y) représentant des viewpoints valides autour du centre.
        """
        viewpoints = []

        for i in range(num_samples):
            theta = np.random.uniform(0, 2 * np.pi)
            r = np.random.uniform(0.8 * radius, radius) # éviter les points trop proches

            # calcul des points 
            x = int(center[0] + r * np.cos(theta))
            y = int(center[1] + r * np.sin(theta))

            if self.is_in_grid(x,y) and self.voxel_grid_map[x][y]==1: # bien vérifié que le point peut être atteignable
                viewpoints.append((x, y))

        return viewpoints


    def _select_best_viewpoint(self, viewpoints, target=None):
        """
        Sélectionne le meilleur point de vue (viewpoint) parmi une liste de points valides,
        en maximisant la visibilité vers la FIS (average_position) et en minimisant
        la distance au drone.

        La sélection est basée sur deux critères :
        - La distance entre le viewpoint et le drone (priorité à la proximité).
        - L’angle ou la distance à la position moyenne de la FIS (meilleure vue).

        Paramètres
        ----------
        viewpoints : list of tuple
            Liste de positions (x, y) valides autour de la FIS.
        target : tuple or None
            Position centrale de la FIS (x, y) à regarder. Si None, aucune pondération angulaire.

        Retourne
        -------
        tuple or None
            Le viewpoint optimal sous forme (x, y), ou None si la liste est vide.
        """

        if not viewpoints:
            return None

        drone_x = self.get_position('x')
        drone_y = self.get_position('y')

        if target is None:
            # Par défaut : choisir le plus proche du drone
            return min(viewpoints, key=lambda vp: np.linalg.norm([vp[0] - drone_x, vp[1] - drone_y]))

        best_score = float('inf')
        best_vp = None
        for vp_x, vp_y in viewpoints:
            dist_to_drone = np.linalg.norm([vp_x - drone_x, vp_y - drone_y])
            dist_to_target = np.linalg.norm([vp_x - target[0], vp_y - target[1]])

            # Score combiné : pondérer distance au drone + éloignement du target (FIS)
            score = dist_to_drone + 0.5 * dist_to_target

            if score < best_score:
                best_score = score
                best_vp = (vp_x, vp_y)

        return best_vp

    def _idx_optimal_FIS(self,F):

        """
            Calcule l'indice optimal d'insertion d'un objet FIS dans la liste `self.list_FIS`, 
            en fonction de la distance entre le drone et le viewpoint du FIS.

            L'insertion est ordonnée de façon croissante par distance au drone, 
            de sorte que les FIS les plus proches soient explorés en priorité.

            Paramètres
            ----------
            F : FIS
                L'objet FIS pour lequel on souhaite déterminer l'indice d'insertion optimal.

            Retourne
            -------
            int
                L'indice auquel insérer le FIS dans `self.list_FIS`.
                Si le FIS ne possède pas de viewpoint, retourne 0 par défaut.
        """
        
        # Trouve l'index d'insertion selon la distance
        drone_x = self.get_position('x')
        drone_y = self.get_position('y')

        insert_idx = 0

        vp_x, vp_y = F.viewpoints
        dist_to_drone = np.linalg.norm([vp_x - drone_x, vp_y - drone_y])

        # Insertion triée
        for i, existing_FIS in enumerate(self.list_FIS):

            if existing_FIS.viewpoints:
                ex_vp_x, ex_vp_y = existing_FIS.viewpoints
                ex_dist = np.linalg.norm([ex_vp_x - drone_x, ex_vp_y - drone_y])
                if dist_to_drone < ex_dist:
                    break
                insert_idx += 1

        return insert_idx

# === Méthodes déplacement et rotation ===

    def reach_FIS(self):

        """
            Tente de rejoindre le viewpoint de la première FIS de la liste (la plus proche).
        """

        if len(self.list_FIS) > 0:
            fis = self.list_FIS[0] # TO DO

            # meilleur point de vu pour couvrir
            if fis.viewpoints:
                (px, py) = fis.viewpoints

                # position moyenne (pax, pay) de la FIS
                pax, pay = fis.average_position[0], fis.average_position[1]
                self.move(px, py, pax, pay, fis)

    def move(self,px,py,pax,pay,FIS):

        """
            Déplace le drone pas à pas depuis sa position actuelle jusqu’au point (px, py) sur la grille grâce à A*,
            puis tourne le drone vers la position moyenne (pax, pay) de la FIS et supprime cette FIS de la liste.

            Paramètres
            ----------
            px, py : int
                Coordonnées sur la grille vers lesquelles se déplacer (point de vue).
            pax, pay : float
                Coordonnées dans le monde de la position moyenne de la FIS.
            FIS : objet
                L’objet Frontière d’Information Spatiale à atteindre et supprimer après déplacement.
        """

        # on récupère la pos du drone dans la grid 
        xd, yd = self.get_position('x'), self.get_position('y')
        xi,yi = world_to_grid(xd,yd,self.voxel_grid_map.shape)

        # on applique algo A* pour trouver le chemin entre (xi,yi) et (px,py)
        l = a_star_grid(self.voxel_grid_map,(xi,yi),(px,py))


        yaw = self._compute_yaw_to_point(px,py)
        self.turn_smooth(yaw)

        # on parcourt ce chemin en faisant bouger le drone
        if l:
            for pix,piy in l:

                # on se déplace au point pix,piy
                pos, ori = p.getBasePositionAndOrientation(self.drone_id)
                pixr,piyr = grid_to_world(pix,piy,self.voxel_grid_map.shape)
                new_pos = [pixr, piyr, pos[2]]
                p.resetBasePositionAndOrientation(self.drone_id, new_pos, ori)

                image = self.update_voxel_grid_map()
                cv2.imshow("Map", image)
                p.stepSimulation()


            # une fois arriver au viewpoint on regarde la position moyenne de la FIS. 
            yaw = self._compute_yaw_to_point(pax,pay)
            self.turn_smooth(yaw)
        self.remove_FIS(FIS)

    def turn_smooth(self, yaw):
        """
            Effectue une rotation progressive du drone autour de son axe vertical (yaw).

            Paramètres
            ----------
            angle : float
                Angle en radians de la rotation désirée.
            steps : int, optionnel
                Nombre d’étapes dans la rotation progressive (par défaut 15).
        """

        max_steps = 30
        min_steps = 5  # pour éviter steps=0
        angle = abs(yaw)
        steps = int(min_steps + (max_steps - min_steps) * (angle / math.pi))

        drone_pos, ori = p.getBasePositionAndOrientation(self.drone_id)
        euler_angles = list(p.getEulerFromQuaternion(ori))
        current_yaw = euler_angles[2]

        # rotation progressive du yaw entre la position actuelle et la cible
        for i in range(1, steps + 1):

            interpolated_yaw = current_yaw + (angle * i / steps)
            new_ori = p.getQuaternionFromEuler([euler_angles[0], euler_angles[1], interpolated_yaw])
            p.resetBasePositionAndOrientation(self.drone_id, drone_pos, new_ori)
            
            # Mettre à jour la profondeur et la carte après chaque étape de rotation
            _ = self.compute_depth_camera_image()
            image1 = self.update_voxel_grid_map()
            
            cv2.imshow("Map", image1)
            p.stepSimulation()  # avancer la simulation
            


# --- Private Helpers ---

    def _compute_yaw_to_point(self, x_target, y_target):
        """
        Calcule l'angle de rotation (yaw) que le drone doit effectuer pour faire face à un point cible.

        Paramètres
        ----------
        x_target : int
            Coordonnée x du point cible dans la grille.
        y_target : int
            Coordonnée y du point cible dans la grille.

        Retourne
        -------
        float
            Angle (en radians) que le drone doit tourner (positif ou négatif) pour s'orienter vers le point cible.
        """

        # récupérer la position actuelle et sa rotation du drone dans l’espace monde
        position, _ = p.getBasePositionAndOrientation(self.drone_id)
        yaw_d = self.get_orientation()  

        x_drone, y_drone = position[0], position[1]

        # convertir les coordonnées grille -> monde pour le point cible
        x_target, y_target = grid_to_world(x_target, y_target,self.voxel_grid_map.shape)

        # calcul du vecteur direction entre drone et cible
        delta_x = x_target - x_drone
        delta_y = y_target - y_drone

        # angle pour faire face à la cible
        true_yaw = math.atan2(delta_y, delta_x) - yaw_d

        return true_yaw



# === Méthodes pour lié aux images === 

    def compute_depth_camera_image(self):

        """
            Capture et calcule une image de profondeur à partir de la caméra virtuelle du drone.

            Cette méthode :
            - Récupère la position et l'orientation du drone dans l'environnement simulé.
            - Calcule la matrice de vue et la matrice de projection pour la caméra.
            - Interroge l'API de simulation pour obtenir l'image de profondeur.
            - Convertit le tampon de profondeur en distances réelles en mètres.
            - Normalise la profondeur pour générer une image 8 bits affichable.
            - Stocke la matrice de profondeur en mètres dans l'attribut `depth_matrix`.

            Retourne
            -------
            np.ndarray
                Image en niveaux de gris (uint8) représentant la profondeur normalisée, prête à être affichée.
        """
            
        aspect = self.width / self.height
        near, far = 0.1, self.max_range  # Near and far clipping plane


        # Get drone's position and orientation
        drone_pos, drone_orient = p.getBasePositionAndOrientation(self.drone_id)

        # Convert quaternion to rotation matrix
        rot_matrix = p.getMatrixFromQuaternion(drone_orient)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Camera offset from the drone
        camera_offset = np.array([0, 0, 0.2])  # Adjust based on drone model

        # Compute camera position
        camera_pos = np.array(drone_pos) + rot_matrix @ camera_offset

        # Define camera direction
        forward_vec = rot_matrix @ np.array([1, 0, 0])
        up_vec = rot_matrix @ np.array([0, 0, 1])

        # Compute view matrix
        view_matrix = p.computeViewMatrix(camera_pos, camera_pos + forward_vec, up_vec)

        # Compute projection matrix
        proj_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, near, far)

        # Get images from the camera
        _, _, _, depth, _ = p.getCameraImage(self.width, self.height, view_matrix, proj_matrix)

        depth_buffer = np.reshape(depth, (self.height, self.width))
        # Convert depth buffer to meters
        depth_matrix = far * near / (far - (far - near) * depth_buffer)

        # Je crois que c'est en mètre... Je ne suis pas sûr. Dans tous les cas on doit la ramener entre 0 et 255 pour l'afficher
        depth_image = (
                (depth_matrix - np.min(depth_matrix))
                / (np.max(depth_matrix) - np.min(depth_matrix))
                * 255
        ).astype(np.uint8)

        # on conserve la profondeur
        self.depth_matrix = np.array(depth_matrix)
        
        return depth_image

    def update_image(self,print_AABB=False):

        """
            Génère une image RGB représentant la carte voxel actuelle avec les obstacles, zones libres et frontières (FIS).

            Cette méthode colore les cellules libres en gris, les obstacles en vert et 
            colorie les clusters FIS avec des couleurs aléatoires, marque leur position moyenne en rouge.

            Paramètres
            ----------
            print_AABB : bool, optionnel
                Si vrai, affiche les bordures des boîtes englobantes autour des FIS (défaut False).

            Retourne
            -------
            np.ndarray
                Image RGB (uint8) agrandie représentant la carte voxel avec annotations.
        """
        
        # Création de l'image avec les dimensions appropriées
        tab = self.voxel_grid_map
        image = np.zeros((len(tab[0]), len(tab[1]), 3), dtype=np.uint8)
        
        # Assignation des couleurs selon les valeurs dans tab
        image[tab == 1] = [100, 100, 100]  # Gris pour libre
        image[tab == 2] = [0, 255, 0]      # Vert pour obstacle
        # Parcours de la liste FIS pour mettre à jour la couleur des cellules
        for F in self.list_FIS:
            # Générer une couleur aléatoire pour chaque F
            random_color = np.random.randint(0, 256, size=3)  # Couleur aléatoire RGB
            
            for x, y in F.cells:
                # Mise à jour de la couleur de la cellule avec la couleur aléatoire
                image[x][y] = random_color

            x,y = int(F.average_position[0]),int(F.average_position[1])
            image[x][y] = [255, 0, 0] 

            if F.aabb is not None and print_AABB:
                (min_i, min_j), (max_i, max_j) = F.aabb
                min_i, min_j = int(min_i), int(min_j)
                max_i, max_j = int(max_i), int(max_j)

                # Bordure de même couleur que random_color
                for j in range(min_j, max_j + 1):
                    image[min_i, j] = random_color  # haut
                    image[max_i, j] = random_color  # bas
                for i in range(min_i, max_i + 1):
                    image[i, min_j] = random_color  # gauche
                    image[i, max_j] = random_color  # droite
        zoom_factor = 3
        image = cv2.resize(image, (image.shape[1] * zoom_factor, image.shape[0] * zoom_factor), interpolation=cv2.INTER_NEAREST)
        return image
    


# === Autres méthodes ===

    def is_in_grid(self, i, j):
        """
        Vérifie si la cellule (i, j) est dans les limites de la carte.
        """
        height, width = self.voxel_grid_map.shape
        return 0 <= i < height and 0 <= j < width

# === Fonctions des positions dans la map réelle et la grille ===

def world_to_grid(x,y,l,center_x=0,center_y=0):
    """

        Convertit une position en coordonnées monde (x, y) en indices (i, j) dans une grille 2D.

        Paramètres
        ----------
        x : float
            Coordonnée x dans le référentiel monde.
        y : float
            Coordonnée y dans le référentiel monde.
        l : tuple(int, int)
            Taille de la grille (nombre de lignes m, nombre de colonnes n).
        center_x : float, optionnel
            Coordonnée x du centre de la grille dans le référentiel monde (défaut 0).
        center_y : float, optionnel
            Coordonnée y du centre de la grille dans le référentiel monde (défaut 0).

        Retourne
        -------
        tuple(int, int)
            Indices (i, j) correspondants à la position dans la grille.
    """

    (m,n) = l 
    i = int(m // 2 + (x-center_x) / resolution)
    j = int(n // 2 + (y-center_y) / resolution)

    i = max(0, min(i, m - 1))
    j = max(0, min(j, n - 1))

    return i, j

def grid_to_world(xi, yi,l):
    """

        Convertit des indices de grille (xi, yi) en coordonnées dans le référentiel monde (x, y).

        Paramètres
        ----------
        xi : int
            Indice ligne dans la grille.
        yi : int
            Indice colonne dans la grille.
        l : tuple(int, int)
            Taille de la grille (nombre de lignes m, nombre de colonnes n).

        Retourne
        -------
        tuple(float, float)
            Coordonnées (x, y) dans le référentiel monde correspondant aux indices de la grille.
    """    
    
    (m,n) = l 
    x = (xi - m // 2) * resolution
    y = (yi - n // 2) * resolution
    return x, y

def compute_new_points(x,y,dist,angle,shape_map):

    """
        Calcule les indices de grille correspondant à un point situé à une distance et un angle donnés 
        à partir d'une position initiale dans le monde réel.

        Paramètres
        ----------
        x : float
            Coordonnée x de la position initiale dans le monde réel.
        y : float
            Coordonnée y de la position initiale dans le monde réel.
        dist : float
            Distance à partir du point initial pour calculer le nouveau point.
        angle : float
            Angle (en radians) par rapport à l'axe x positif indiquant la direction du nouveau point.
        shape_map : tuple(int, int)
            Dimensions (m, n) de la grille voxel, utilisées pour convertir les coordonnées monde en indices de grille.

        Retourne
        -------
        tuple(int, int)
            Indices (xi, yi) du point calculé dans la grille voxel.
    """
        
    # on fait la projection
    new_x = x + dist * np.cos(angle)
    new_y = y + dist * np.sin(angle)
    xi, yi = world_to_grid(new_x, new_y,shape_map)
    return (xi,yi)

def isFree(p):

    """ Vérifie que le point est libre """
    return p!=2

# === Fonctions de déplacement (Algorithme A*) ===

def dist_eucli(a, b):
    """ Calcul la distance euclienne entre les points a et b"""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def a_star_grid(matrix, start, goal):

    """
        Implémente l'algorithme A* pour trouver le plus court chemin dans une grille 2D.

        Paramètres
        ----------
        matrix : list of list of int
            Grille 2D représentant l'environnement, où 2 indique un obstacle, 1 une case libre.
        start : tuple(int, int)
            Coordonnées (x, y) du point de départ dans la grille.
        goal : tuple(int, int)
            Coordonnées (x, y) du point d'arrivée dans la grille.

        Retourne
        -------
        list of tuple(int, int) or None
            Liste des positions formant le chemin du départ à l'arrivée, 
            ou None s'il n'existe pas de chemin.
    """
    rows, cols = len(matrix), len(matrix[0])

    # Ensemble de positions à explorer, avec leur f_score
    open_set = []
    heapq.heappush(open_set, (0, start))

    # Pour reconstruire le chemin une fois l'arrivée atteinte
    came_from = {}

    # Coût pour arriver à chaque case
    g_score = {start: 0}

    # Estimation du coût total (f = g + h)
    f_score = {start: dist_eucli(start, goal)}

    # 8 directions de mouvement
    directions = [
    (-1, 0), (1, 0), (0, -1), (0, 1), # haut, bas, gauche, droite
    (-1, -1), (-1, 1), (1, -1), (1, 1) # diagonales
    ]

    step = 0
    while open_set:
        # Prendre le noeud avec le plus petit f_score
        _, current = heapq.heappop(open_set)

        step+=1

        if step==1000:
            return None
        
        if current == goal:
        # Reconstruire le chemin
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1] # chemin du départ à l'arrivée

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            x, y = neighbor

            # Vérifie que le voisin est dans la grille
            if 0 <= x < cols and 0 <= y < rows:
                if matrix[y][x] == 2:
                    continue # obstacle

            # Vérifie qu'on ne coupe pas un coin
            if dx != 0 and dy != 0:
            # Si diagonale, on vérifie les cases adjacentes
                if 0<= current[0] + dx and current[0] + dx < cols \
                    and 0<= current[1] and current[1] < rows and matrix[current[1]][current[0] + dx] == 2 or \
                0<= current[1] + dy  and current[1] + dy < rows and 0<= current[0] < cols and current[0] < cols and matrix[current[1] + dy][current[0]] == 2:
                    continue # coin bloqué

            # Coût de déplacement (1 pour cardinaux, √2 pour diagonales)
            move_cost = math.hypot(dx, dy)
            tentative_g = g_score[current] + move_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + dist_eucli(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

def find_clusters(L: list, len_map, max_range=0.3):
    """
    Regroupe des cellules (indices de grille) proches spatialement en clusters.

    Parameters
    ----------
    L : list of tuple
        Liste de coordonnées (xi, yi) dans la grille représentant des cellules candidates à regrouper.
    len_map : tuple
        Dimensions (m, n) de la grille, utilisées pour la conversion en coordonnées monde.
    max_range : float, optional
        Distance maximale (en mètres) entre deux cellules pour qu'elles soient considérées dans le même cluster (default: 0.3m).

    Returns
    -------
    clusters : list of list of tuple
        Liste de clusters, chacun étant une liste de cellules (xi, yi) regroupées ensemble.
    """

    clusters = []
    not_seen = L.copy()  # cellules à explorer

    while not_seen:
        # On part d'un point de départ non encore visité
        xi, yi = not_seen.pop()
        px, py = grid_to_world(xi, yi, len_map)  # coordonnées monde
        new_cluster = [(xi, yi)]  # nouveau cluster initialisé

        # Parcours des autres cellules restantes
        for xi2, yi2 in not_seen:
            px2, py2 = grid_to_world(xi2, yi2, len_map)
            # Si la distance est inférieure à la distance seuil
            if np.linalg.norm([px - px2, py - py2]) <= max_range:
                if (xi2, yi2) not in new_cluster:
                    new_cluster.append((xi2, yi2))

        clusters.append(new_cluster)

        # Mise à jour des cellules restantes
        not_seen_set = set(not_seen)
        cluster_set = set(new_cluster)
        not_seen = list(not_seen_set - cluster_set)

    return clusters

