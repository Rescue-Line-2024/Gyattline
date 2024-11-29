import os
import random

current_dir = os.path.dirname(os.path.abspath(__file__))
# Directory del dataset
images_dir = f"{current_dir}/images"
labels_dir = f"{current_dir}/labels"
output_dir = current_dir

# Percorsi dei file di output
train_txt = os.path.join(output_dir, "train.txt")
val_txt = os.path.join(output_dir, "val.txt")

# Percentuale di dati per il validation set
val_split = 0.2

# Ottieni tutte le immagini
images = [f for f in os.listdir(images_dir) if f.endswith(".jpg")]

# Mescola le immagini in modo casuale
random.shuffle(images)

# Dividi in train e val
split_idx = int(len(images) * (1 - val_split))
train_images = images[:split_idx]
val_images = images[split_idx:]

# Salva i percorsi delle immagini nei file train.txt e val.txt
with open(train_txt, "w") as train_file:
    for img in train_images:
        train_file.write(os.path.join(images_dir, img) + "\n")

with open(val_txt, "w") as val_file:
    for img in val_images:
        val_file.write(os.path.join(images_dir, img) + "\n")

print("Dataset diviso in train e val!")
print(f"Train set: {len(train_images)} immagini")
print(f"Validation set: {len(val_images)} immagini")
