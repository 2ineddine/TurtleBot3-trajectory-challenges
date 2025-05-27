import random
import xml.etree.ElementTree as ET
import os


# Déterminer le chemin absolu du fichier SDF
script_dir = os.path.dirname(os.path.abspath(__file__))  # Récupère le dossier du script
sdf_file = os.path.join(script_dir, "model_avant_random.sdf")  # Construit le chemin absolu

# Vérifier si le fichier existe avant de continuer
if not os.path.exists(sdf_file):
    raise FileNotFoundError(f"Fichier SDF non trouvé : {sdf_file}")


# Charger le fichier SDF

tree = ET.parse(sdf_file)
root = tree.getroot()

# Définir une nouvelle position aléatoire
new_theta = round(random.uniform(1, 2.4), 3)
new_x = round(random.uniform(0.5, 0.75), 3)
new_y = round(random.uniform(-0.75, -0.25), 3)


pose_element = root.find(".//pose")
if pose_element is not None:
    pose_values = pose_element.text.split()
    pose_values[5] = str(new_theta)  # Modifier uniquement y
    pose_values[0] = str(new_x)  # Modifier uniquement y
    pose_values[1] = str(new_y)  # Modifier uniquement y
    pose_element.text = " ".join(pose_values)

# Sauvegarder les modifications
modified_sdf_file = os.path.join(script_dir, "model.sdf")
tree.write(modified_sdf_file)


