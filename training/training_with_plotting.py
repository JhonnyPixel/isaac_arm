# train_with_metrics.py
import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import matplotlib.pyplot as plt
from math import sqrt

# ------------------------
# Config
# ------------------------
DATA_PATH = "training_data_8.npz"   # file atteso con 'inputs' (N,8) e 'targets' (N,8)
MODEL_PATH = "model_weights.pth"
HISTORY_PATH = "training_history.npz"
OUT_DIR = "training_plots"
os.makedirs(OUT_DIR, exist_ok=True)

INPUT_DIM = 8
OUTPUT_DIM = 8   # 7 giunti + 1 grip
BATCH_SIZE = 64
LR = 1e-3
EPOCHS = 100
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ------------------------
# Simple MLP (same structure as tuo codice, output dim aggiornato)
# ------------------------
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

# ------------------------
# Load data, split, normalize inputs
# ------------------------
data = np.load(DATA_PATH)
X_all = data['inputs']    # (N, 8)
Y_all = data['targets']   # (N, 8)
assert X_all.ndim == 2 and Y_all.ndim == 2
assert X_all.shape[1] == INPUT_DIM
assert Y_all.shape[1] == OUTPUT_DIM

# Shuffle + split
N = X_all.shape[0]
perm = np.random.permutation(N)
train_ratio = 0.8
n_train = int(N * train_ratio)
train_idx = perm[:n_train]
val_idx = perm[n_train:]

X_train = X_all[train_idx]
Y_train = Y_all[train_idx]
X_val = X_all[val_idx]
Y_val = Y_all[val_idx]

# Normalize inputs (fit on train)
X_mean = X_train.mean(axis=0)
X_std = X_train.std(axis=0)
# avoid zero std
X_std[X_std == 0] = 1.0

X_train_n = (X_train - X_mean) / X_std
X_val_n = (X_val - X_mean) / X_std

# Convert to tensors and dataloaders
tensor_X_train = torch.tensor(X_train_n, dtype=torch.float32)
tensor_Y_train = torch.tensor(Y_train, dtype=torch.float32)
tensor_X_val = torch.tensor(X_val_n, dtype=torch.float32)
tensor_Y_val = torch.tensor(Y_val, dtype=torch.float32)

train_ds = TensorDataset(tensor_X_train, tensor_Y_train)
val_ds = TensorDataset(tensor_X_val, tensor_Y_val)
train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)

# ------------------------
# Utils: metrics
# ------------------------
def mse_numpy(y_true, y_pred):
    return np.mean((y_true - y_pred) ** 2)

def rmse_numpy(y_true, y_pred):
    return np.sqrt(mse_numpy(y_true, y_pred))

def mae_numpy(y_true, y_pred):
    return np.mean(np.abs(y_true - y_pred))

# ------------------------
# Training
# ------------------------
model = ArmMLP(INPUT_DIM, OUTPUT_DIM).to(DEVICE)
criterion = nn.MSELoss(reduction='mean')
optimizer = optim.Adam(model.parameters(), lr=LR)

history = {
    'train_loss': [],
    'val_loss': [],
    'val_mae': [],
    'val_rmse': [],
    'val_per_joint_mae': [],   # list of arrays (num_epochs x OUTPUT_DIM)
    'val_per_joint_rmse': []
}

for epoch in range(1, EPOCHS+1):
    model.train()
    running_loss = 0.0
    batches = 0
    for xb, yb in train_loader:
        xb = xb.to(DEVICE)
        yb = yb.to(DEVICE)
        pred = model(xb)
        loss = criterion(pred, yb)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        running_loss += loss.item()
        batches += 1

    avg_train_loss = running_loss / batches if batches > 0 else 0.0
    history['train_loss'].append(avg_train_loss)

    # Validation
    model.eval()
    with torch.no_grad():
        val_preds = []
        val_trues = []
        for xb, yb in val_loader:
            xb = xb.to(DEVICE)
            pred = model(xb).cpu().numpy()
            val_preds.append(pred)
            val_trues.append(yb.numpy())
        val_preds = np.vstack(val_preds)
        val_trues = np.vstack(val_trues)

    val_mse = mse_numpy(val_trues, val_preds)
    val_rmse = rmse_numpy(val_trues, val_preds)
    val_mae = mae_numpy(val_trues, val_preds)
    # per-joint
    per_joint_mae = np.mean(np.abs(val_trues - val_preds), axis=0)      # shape (OUTPUT_DIM,)
    per_joint_rmse = np.sqrt(np.mean((val_trues - val_preds)**2, axis=0))

    history['val_loss'].append(val_mse)
    history['val_mae'].append(val_mae)
    history['val_rmse'].append(val_rmse)
    history['val_per_joint_mae'].append(per_joint_mae)
    history['val_per_joint_rmse'].append(per_joint_rmse)

    print(f"Epoch {epoch:03d}  TrainLoss={avg_train_loss:.6f}  ValMSE={val_mse:.6f}  ValMAE={val_mae:.6f}")

# Save model + history + normalization stats
torch.save(model.state_dict(), MODEL_PATH)
np.savez(HISTORY_PATH,
         train_loss=np.array(history['train_loss']),
         val_loss=np.array(history['val_loss']),
         val_mae=np.array(history['val_mae']),
         val_rmse=np.array(history['val_rmse']),
         val_per_joint_mae=np.array(history['val_per_joint_mae']),
         val_per_joint_rmse=np.array(history['val_per_joint_rmse']),
         x_mean=X_mean, x_std=X_std,
         input_dim=INPUT_DIM, output_dim=OUTPUT_DIM)

# ------------------------
# Plotting (matplotlib; single plot per figure)
# ------------------------
# 1) Loss curve (train vs val)
plt.figure()
plt.plot(history['train_loss'], label='train_loss')
plt.plot(history['val_loss'], label='val_mse')
plt.xlabel('Epoch')
plt.ylabel('MSE')
plt.title('Training and Validation Loss')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, 'loss_curve.png'))
plt.close()

# 2) Val MAE / RMSE over epochs
plt.figure()
plt.plot(history['val_mae'], label='val_mae')
plt.plot(history['val_rmse'], label='val_rmse')
plt.xlabel('Epoch')
plt.ylabel('Error')
plt.title('Validation MAE and RMSE')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, 'val_mae_rmse.png'))
plt.close()

# 3) Per-joint MAE bar (last epoch)
per_joint_mae_last = history['val_per_joint_mae'][-1]
plt.figure()
plt.bar(np.arange(OUTPUT_DIM), per_joint_mae_last)
plt.xlabel('Joint index')
plt.ylabel('MAE')
plt.title('Per-joint MAE (validation, last epoch)')
plt.savefig(os.path.join(OUT_DIR, 'per_joint_mae.png'))
plt.close()

# 4) Per-joint RMSE bar (last epoch)
per_joint_rmse_last = history['val_per_joint_rmse'][-1]
plt.figure()
plt.bar(np.arange(OUTPUT_DIM), per_joint_rmse_last)
plt.xlabel('Joint index')
plt.ylabel('RMSE')
plt.title('Per-joint RMSE (validation, last epoch)')
plt.savefig(os.path.join(OUT_DIR, 'per_joint_rmse.png'))
plt.close()

# 5) Histogram of absolute errors (all joints concatenated, last epoch predictions)
abs_errors = np.abs(val_trues - val_preds).reshape(-1)
plt.figure()
plt.hist(abs_errors, bins=80)
plt.xlabel('Absolute error')
plt.ylabel('Count')
plt.title('Histogram of absolute errors (validation)')
plt.savefig(os.path.join(OUT_DIR, 'abs_error_hist.png'))
plt.close()

# 6) Scatter predicted vs true for joint 0 (example)
joint_idx = 0
plt.figure()
plt.scatter(val_trues[:, joint_idx], val_preds[:, joint_idx], s=6)
plt.xlabel('True joint value')
plt.ylabel('Predicted joint value')
plt.title(f'Predicted vs True (joint {joint_idx})')
# plot identity line
mn = min(val_trues[:, joint_idx].min(), val_preds[:, joint_idx].min())
mx = max(val_trues[:, joint_idx].max(), val_preds[:, joint_idx].max())
plt.plot([mn, mx], [mn, mx])
plt.savefig(os.path.join(OUT_DIR, f'scatter_joint{joint_idx}.png'))
plt.close()

# 7) Boxplot of per-joint absolute errors
per_joint_abs_errors = np.mean(np.abs(val_trues - val_preds), axis=0)  # for boxplot we prefer per-sample arrays
# build list of per-joint arrays (per-sample)
per_joint_arrays = [np.abs(val_trues[:, j] - val_preds[:, j]) for j in range(OUTPUT_DIM)]
plt.figure()
plt.boxplot(per_joint_arrays, labels=[str(i) for i in range(OUTPUT_DIM)])
plt.xlabel('Joint index')
plt.ylabel('Absolute error')
plt.title('Per-joint absolute error distribution (validation)')
plt.savefig(os.path.join(OUT_DIR, 'per_joint_boxplot.png'))
plt.close()

print("Training finished. Model + history saved. Plots in folder:", OUT_DIR)
