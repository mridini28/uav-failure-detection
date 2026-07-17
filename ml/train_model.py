import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
import joblib

EPSILON = 1e-30  # avoid log(0)

def add_log_features(df):
    df = df.copy()
    df['log_lat_var'] = np.log10(df['gps_lat_variance'] + EPSILON)
    df['log_lon_var'] = np.log10(df['gps_lon_variance'] + EPSILON)
    return df

# Load normal flight data
df = pd.read_csv('/home/mitta/px4_ros_ws/flight_data_normal_v2.csv')
df = df[df['gps_window_filled'] == 1]
df = add_log_features(df)

feature_cols = ['log_lat_var', 'log_lon_var']
X_train = df[feature_cols].values

print(f"Training on {len(X_train)} rows of normal flight data")
print(df[feature_cols].describe())

model = IsolationForest(
    n_estimators=100,
    contamination=0.01,
    random_state=42
)
model.fit(X_train)

joblib.dump(model, '/home/mitta/px4_ros_ws/ml/anomaly_model.joblib')
print("\nModel saved to ~/px4_ros_ws/ml/anomaly_model.joblib")

# --- Evaluate against the frozen-fault dataset ---
df_frozen = pd.read_csv('/home/mitta/px4_ros_ws/flight_data_frozen.csv')
df_frozen = df_frozen[df_frozen['gps_window_filled'] == 1]
df_frozen = add_log_features(df_frozen)

X_frozen = df_frozen[feature_cols].values
predictions = model.predict(X_frozen)
scores = model.decision_function(X_frozen)
df_frozen['anomaly_score'] = scores
df_frozen['predicted_anomaly'] = predictions == -1

n_anomalies = (predictions == -1).sum()
n_total = len(predictions)
print(f"\nEvaluation on frozen-fault dataset:")
print(f"  Flagged as anomalous: {n_anomalies}/{n_total} rows ({100*n_anomalies/n_total:.1f}%)")

chunk_size = len(df_frozen) // 10
print("\nOver time (chunked in 10 groups):")
for i in range(10):
    start = i * chunk_size
    end = start + chunk_size
    chunk = df_frozen.iloc[start:end]
    pct_flagged = chunk['predicted_anomaly'].mean() * 100
    mean_score = chunk['anomaly_score'].mean()
    print(f"  Rows {start}-{end}: {pct_flagged:.0f}% flagged, mean score = {mean_score:.4f}")