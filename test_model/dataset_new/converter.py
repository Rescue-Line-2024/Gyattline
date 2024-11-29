import os
import shutil
import xml.etree.ElementTree as ET

# Directory contenente i file XML e JPG
input_dir = "..//balls_dataset"  # Sostituisci con il percorso della tua directory
current_dir = os.path.dirname(os.path.abspath(__file__))

output_images_dir = os.path.join(current_dir, "images")
output_labels_dir = os.path.join(current_dir, "labels")

# Definizione delle classi
classes = ["silver_ball", "black_ball"]  # Sostituisci con le tue classi

# Assicurati che le cartelle di output esistano
os.makedirs(output_images_dir, exist_ok=True)
os.makedirs(output_labels_dir, exist_ok=True)

def convert(size, box):
    """Converte le coordinate Pascal VOC in YOLO format."""
    dw = 1.0 / size[0]
    dh = 1.0 / size[1]
    x = (box[0] + box[1]) / 2.0 - 1
    y = (box[2] + box[3]) / 2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    return (x * dw, y * dh, w * dw, h * dh)

# Processa i file nella directory
for file in os.listdir(input_dir):
    # Gestione delle immagini JPG
    if file.endswith(".jpg"):
        shutil.move(os.path.join(input_dir, file), os.path.join(output_images_dir, file))

    # Gestione dei file XML
    elif file.endswith(".xml"):
        tree = ET.parse(os.path.join(input_dir, file))
        root = tree.getroot()

        # Ottieni dimensioni dell'immagine
        size = root.find("size")
        width = int(size.find("width").text)
        height = int(size.find("height").text)

        # Crea un file TXT corrispondente per ogni XML
        txt_file_path = os.path.join(output_labels_dir, file.replace(".xml", ".txt"))
        with open(txt_file_path, "w") as txt_file:
            for obj in root.findall("object"):
                cls = obj.find("name").text
                if cls not in classes:
                    continue  # Salta le classi non riconosciute

                cls_id = classes.index(cls)
                xml_box = obj.find("bndbox")
                b = (float(xml_box.find("xmin").text), float(xml_box.find("xmax").text),
                     float(xml_box.find("ymin").text), float(xml_box.find("ymax").text))
                bb = convert((width, height), b)
                txt_file.write(f"{cls_id} {' '.join(map(str, bb))}\n")

print("Conversione completata. Le immagini e le annotazioni sono state organizzate.")