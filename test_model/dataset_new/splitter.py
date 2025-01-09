import os
import random
import shutil

# Directory del dataset
current_dir = os.path.dirname(os.path.abspath(__file__))
images_dir = f"{current_dir}/images"
labels_dir = f"{current_dir}/labels"

# Cartelle di output per train e val
train_images_dir = f"{images_dir}/train"
val_images_dir = f"{images_dir}/val"
train_labels_dir = f"{labels_dir}/train"
val_labels_dir = f"{labels_dir}/val"

# Percentuale di dati per il validation set
val_split = 0.2

# Crea le sottocartelle train e val, se non esistono
os.makedirs(train_images_dir, exist_ok=True)
os.makedirs(val_images_dir, exist_ok=True)
os.makedirs(train_labels_dir, exist_ok=True)
os.makedirs(val_labels_dir, exist_ok=True)

# Ottieni tutte le immagini
images = [f for f in os.listdir(images_dir) if f.endswith(".jpg")]

# Mescola le immagini in modo casuale
random.shuffle(images)

# Dividi in train e val
split_idx = int(len(images) * (1 - val_split))
train_images = images[:split_idx]
val_images = images[split_idx:]

# Funzione per spostare immagini e annotazioni
def move_files(image_list, target_images_dir, target_labels_dir):
    for img in image_list:
        # Sposta l'immagine
        img_path = os.path.join(images_dir, img)
        shutil.move(img_path, os.path.join(target_images_dir, img))

        # Sposta il file di annotazione corrispondente (.txt)
        label_path = os.path.join(labels_dir, img.replace(".jpg", ".txt"))
        if os.path.exists(label_path):
            shutil.move(label_path, os.path.join(target_labels_dir, img.replace(".jpg", ".txt")))

# Sposta i file di training
move_files(train_images, train_images_dir, train_labels_dir)

# Sposta i file di validazione
move_files(val_images, val_images_dir, val_labels_dir)

print("Dataset diviso in train e val!")
print(f"Train set: {len(train_images)} immagini")
print(f"Validation set: {len(val_images)} immagini")
