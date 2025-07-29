import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np

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

# === 3. Training loop ===
def train():
    dataset = ArmDataset("training_data.npz")
    loader = DataLoader(dataset, batch_size=64, shuffle=True)

    model = ArmMLP(input_size=8, output_size=7)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(100):
        for inputs, targets in loader:
            pred = model(inputs)
            loss = criterion(pred, targets)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        print(f"Epoch {epoch}, Loss: {loss.item():.4f}")

    torch.save(model.state_dict(), "model_weights.pth")

if __name__ == "__main__":
    train()
