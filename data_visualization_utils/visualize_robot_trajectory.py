import pandas as pd
import matplotlib.pyplot as plt

filepath = "datasets/nicoo/positions.csv"
filepath_geofence = "datasets/nicoo/geofence.csv"
data = pd.read_csv(filepath)

geofence = pd.read_csv(filepath_geofence)

x = data["x"]
y = data["y"]
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(x, y, color="blue", label="Robot Path")


geo_x = geofence["x"]
geo_y = geofence["y"]
ax.plot(geo_x, geo_y, color="red", label="Geofence")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title(f"Robot Path from {filepath}")

plt.legend()
plt.grid(True)
plt.show()
