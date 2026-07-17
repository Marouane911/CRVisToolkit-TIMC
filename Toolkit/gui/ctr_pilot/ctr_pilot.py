from pathlib import Path
ROOT_DIR = Path(__file__).resolve().parents[3]
print(ROOT_DIR)

import camitk
import sys
import csv
import vtk
import numpy as np
from scipy.spatial.transform import Rotation

GUI_DIR = ROOT_DIR / "Toolkit" / "gui"

if str(GUI_DIR) not in sys.path:
    sys.path.append(str(GUI_DIR))

from ctr_solver import CTRSolver
from ctr_loader import CTRLoader
from ctr_vtk_export import CTRVTKExporter
from ctr_data import CTRData
from ctr_solver import CTRSolver
from vtk.util import numpy_support
from ctr_constraints import CTRConstraints


def process(self):

    # camitk.warning("PROCESS CALLED")

    # 1. Lecture brute des translations actuelles
    q_brut = [
        self.getParameterValue("Q0"),
        self.getParameterValue("Q1"),
        self.getParameterValue("Q2")
    ]

    # 2. Sécurité : Création de la mémoire au premier lancement
    if not hasattr(self, 'prev_q'):
        self.prev_q = list(q_brut)

    if not hasattr(self, 'last_valid_q'):
        # On sauvegarde l'état initial (censé être sain) des 6 variables
        q3_init = self.getParameterValue("Q3")
        q4_init = self.getParameterValue("Q4")
        q5_init = self.getParameterValue("Q5")
        self.last_valid_q = q_brut + [q3_init, q4_init, q5_init]

    # 3. Détection : Quel chariot vient d'être manipulé ?
    active_tube = 2 # Externe par défaut
    for i in range(3):
        if abs(q_brut[i] - self.prev_q[i]) > 1e-6:
            active_tube = i
            break

    # 4. Lecture brute des longueurs
    l_tubes = [
        self.getParameterValue("L1"),
        self.getParameterValue("L2"),
        self.getParameterValue("L3")
    ]


    # Garde-fou physique (L'entraînement)
    q_corrige = CTRConstraints.enforce_telescopic_constraints(
        q=q_brut,
        active=active_tube,
        entrainement=True,
        tubes_lengths=l_tubes
    )

    # Rétroaction visuelle : Si la physique a bougé un tube, on met à jour l'interface
    if any(abs(q_brut[i] - q_corrige[i]) > 1e-6 for i in range(3)):
        self.setParameterValue("Q0", float(q_corrige[0]))
        self.setParameterValue("Q1", float(q_corrige[1]))
        self.setParameterValue("Q2", float(q_corrige[2]))
        self.prev_q = list(q_corrige)
    else:
        self.prev_q = list(q_corrige)


    # Affectation finale des variables
    q0, q1, q2 = q_corrige[0], q_corrige[1], q_corrige[2]
    l1, l2, l3 = l_tubes[0], l_tubes[1], l_tubes[2]

    q3 = self.getParameterValue("Q3")
    q4 = self.getParameterValue("Q4")
    q5 = self.getParameterValue("Q5")

    l_k1 = self.getParameterValue("L_k1")
    l_k2 = self.getParameterValue("L_k2")
    l_k3 = self.getParameterValue("L_k3")

    Ux1 = self.getParameterValue("Ux1")
    Ux2 = self.getParameterValue("Ux2")
    Ux3 = self.getParameterValue("Ux3")

    rOut1 = self.getParameterValue("ROut1")
    rOut2 = self.getParameterValue("ROut2")
    rOut3 = self.getParameterValue("ROut3")

    base_x = self.getParameterValue("BaseX")
    base_y = self.getParameterValue("BaseY")
    base_z = self.getParameterValue("BaseZ")

    rot_x = self.getParameterValue("AxeX")
    rot_y = self.getParameterValue("AxeY")
    rot_z = self.getParameterValue("AxeZ")

    ## Écriture des nouveaux paramètres dans parameters_tmp.csv
    csv_path = (
        ROOT_DIR
        / "Toolkit"
        / "python"
        / "parameters_temp.csv"
    )

    # Sauvegarde sur le disque
    with open(csv_path, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    if len(rows) == 0:
        # camitk.warning("parameters_temp.csv vide")
        return False

    # On fait une copie exacte de la ligne qui fonctionnait avant modification
    backup_row = rows[0].copy()

    # Mise à jour du dictionnaire
    rows[0]["l1"] = str(l1)
    rows[0]["l2"] = str(l2)
    rows[0]["l3"] = str(l3)
    rows[0]["l_k1"] = str(l_k1)
    rows[0]["l_k2"] = str(l_k2)
    rows[0]["l_k3"] = str(l_k3)
    rows[0]["Ux1"] = str(Ux1)
    rows[0]["Ux2"] = str(Ux2)
    rows[0]["Ux3"] = str(Ux3)
    rows[0]["rOut1"] = str(rOut1)
    rows[0]["rOut2"] = str(rOut2)
    rows[0]["rOut3"] = str(rOut3)

    fieldnames = rows[0].keys()

    # On écrit la tentative de modification des paramètres dans le fichier CSV
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    # camitk.warning("parameters_temp.csv updated")

    ###### fin de l'écriture

    result = CTRSolver.run(
        root_toolkit=str(ROOT_DIR / "Toolkit"),
        q_values=[q0, q1, q2, q3, q4, q5],
        params_path=csv_path
    )

    # camitk.warning(f"Succès du solveur : {result['success']}")

    if not result["success"]: # CRASH DU SOLVER (Modèle mathématique)

        camitk.warning(f"{str(result)} : restauration des anciens paramètres")

        # On réécrit immédiatement l'ancienne sauvegarde dans le CSV actuel car le CSV acuel est cassé
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(backup_row)

        for i in range(6):
            self.setParameterValue(f"Q{i}", float(self.last_valid_q[i])) # restauration des paramètres de translation et rotation

        # Restauration de la détection de chariot pour le prochain tour
        self.prev_q = list(self.last_valid_q[:3])

        return False

    # camitk.warning("Calcul réussi, mise à jour du mesh ...")
    # Si le calcul a réussi, on met à jour notre référence
    self.last_valid_q = [q0, q1, q2, q3, q4, q5]

    backbone_path = (
        ROOT_DIR
        / "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots"
        / "build"
        / "robot_backbone.txt"
    )

    steps = CTRLoader.load(str(backbone_path))

    if len(steps) == 0:
        # camitk.warning("No backbone data loaded")
        return False

    step = steps[0]
    data = CTRData(
        step["matrix"],
        step["iEnd"],
        step["S"]
    )

    # camitk.warning("BEFORE EXPORT")

    polydata = CTRVTKExporter.export_centerline(
        data.matrix_data,
        data.end_int,
        data.end_mid,
        data.end_ext,
        rOut1,
        rOut2,
        rOut3,
        "/tmp/robot_centerline.vtk"
    )

    # Transformation Globale (translation et rotation)
    transform = vtk.vtkTransform()

    # Application de la transformation sur le maillage 3D
    transform_filter = vtk.vtkTransformPolyDataFilter() # transformer les coordonnées des points
    transform_filter.SetInputData(polydata)
    transform_filter.SetTransform(transform)
    transform_filter.Update()

    # Récupération du maillage après ses transformations de translation et rotation globale
    polydata_transforme = transform_filter.GetOutput()

    # Extraction des points composant le maillage 3D
    points = numpy_support.vtk_to_numpy(
        polydata_transforme.GetPoints().GetData()
    )

    P_tip = points[-1]

    polys = numpy_support.vtk_to_numpy(
        polydata_transforme.GetPolys().GetData()
    ).reshape(-1, 4)


    mesh = None
    image = None

    for target in self.getTargets():

        mesh_component = target.as_type("MeshComponent")
        if mesh_component is not None:
            mesh = mesh_component

        image_component = target.as_type("ImageComponent")
        if image_component is not None:
            image = image_component

    mesh.replacePointSet(points)



    frames = camitk.TransformationManager.getFramesOfReference()

    robot_frame = next(
        f for f in frames
        if f.getName() == "vtk output"
    )

    world_frame = camitk.TransformationManager.getWorldFrame()


    data_frame = next(
        (
            f for f in frames
            if f.getName().endswith("(data)")
        ),
        None
    )

    if data_frame is not None:
        reference_frame = data_frame

    else:
        reference_frame = next(
            (
                f for f in frames
                if f != robot_frame
                and f != world_frame
                and not f.getName().endswith("(main)")
                and not f.getName().endswith("(arbitrary)")
            ),
            world_frame
        )


    # camitk.warning(f"Reference frame = {reference_frame.getName()}")

    if reference_frame != world_frame:

        relation_exists = any(
            T.getFrom() == robot_frame
            and T.getTo() == reference_frame
            for T in camitk.TransformationManager.getTransformations()
        )

        if not relation_exists:
            camitk.TransformationManager.addTransformation(
                robot_frame,
                reference_frame
            )
            camitk.TransformationManager.removeTransformation(
                robot_frame,
                world_frame
            )
        
        reference_world_exists = any(
        T.getFrom() == reference_frame
        and T.getTo() == world_frame
        for T in camitk.TransformationManager.getTransformations()
        )

        if not reference_world_exists:
            camitk.TransformationManager.addTransformation(
                reference_frame,
                world_frame
            )


    rotation = Rotation.from_euler(
        'zyx',
        [rot_z, rot_y, rot_x],
        degrees=True
    )

    T_robot_reference = np.eye(4)

    T_robot_reference[:3, :3] = rotation.as_matrix()

    T_robot_reference[:3, 3] = [
        float(base_x),
        float(base_y),
        float(base_z)
    ]


    camitk.TransformationManager.updateTransformation(
        robot_frame,
        reference_frame,
        T_robot_reference.tolist()
    )

    camitk.refresh()

    return True



