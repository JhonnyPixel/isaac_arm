import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import matplotlib.pyplot as plt

# === 1. Dataset ===
class ArmDataset(Dataset):
    def __init__(self, data_path):
        data = np.load(data_path)
        self.X = torch.tensor(data['inputs'], dtype=torch.float32)
        self.Y = torch.tensor(data['targets'], dtype=torch.float32)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]

# === 2. Rete neurale ===
class ArmMLP(nn.Module):
    def __init__(self, input_size, output_size):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_size)
        )

    def forward(self, x):
        return self.net(x)

# === 3. Training con validazione ===
def train(train_path, val_path, input_dim=8, output_dim=8, epochs=100, batch_size=64, lr=1e-3):
    # Dataset e dataloader
    train_dataset = ArmDataset(train_path)
    val_dataset = ArmDataset(val_path)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # Modello
    model = ArmMLP(input_dim, output_dim)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # Per salvare le metriche
    train_losses = []
    val_losses = []

    for epoch in range(epochs):
        # Training
        model.train()
        epoch_train_loss = 0
        for inputs, targets in train_loader:
            pred = model(inputs)
            loss = criterion(pred, targets)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_train_loss += loss.item()

        avg_train_loss = epoch_train_loss / len(train_loader)
        train_losses.append(avg_train_loss)

        # Validation
        model.eval()
        epoch_val_loss = 0
        with torch.no_grad():
            for inputs, targets in val_loader:
                pred = model(inputs)
                loss = criterion(pred, targets)
                epoch_val_loss += loss.item()

        avg_val_loss = epoch_val_loss / len(val_loader)
        val_losses.append(avg_val_loss)

        print(f"Epoch {epoch+1}/{epochs} | Train Loss: {avg_train_loss:.4f} | Val Loss: {avg_val_loss:.4f}")

    # Salva modello
    torch.save(model.state_dict(), "model_weights.pth")

    # Plot delle curve di training/validation
    plt.figure(figsize=(8, 6))
    plt.plot(train_losses, label="Training Loss")
    plt.plot(val_losses, label="Validation Loss")
    plt.xlabel("Epoche")
    plt.ylabel("MSE Loss")
    plt.title("Andamento della perdita durante l'allenamento")
    plt.legend()
    plt.grid(True)
    plt.show()

    return model, train_losses, val_losses

if __name__ == "__main__":
    train("training_data_8.npz", "validation_data.npz")

