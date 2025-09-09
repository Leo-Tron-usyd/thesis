import numpy as np
import math
import matplotlib.pyplot as plt

# Given parameters
BC = 750
L2 = 600

# Lists to store valid points
A_points = []
E_points = []

# Check collision function
def check_collisions(joint_positions):
    def distance_point_to_segment(A, B, P):
        AB = np.array([B[0] - A[0], B[1] - A[1]])
        AP = np.array([P[0] - A[0], P[1] - A[1]])
        
        if np.dot(AB, AB) == 0:
            return np.linalg.norm(AP)
        
        t = np.dot(AP, AB) / np.dot(AB, AB)
        t = np.clip(t, 0, 1)
        
        C = A + t * AB
        return np.linalg.norm(C - P)

    P = np.array([400, 0])

    for i in range(len(joint_positions) - 1):
        A = np.array(joint_positions[i])
        B = np.array(joint_positions[i + 1])
        distance = distance_point_to_segment(A, B, P)
        if distance < 300:
            return True
    return False

# Generate valid points
for alpha in np.arange(0, 0.5*math.pi, 0.01):
    for L3 in np.arange(460, 500, 5):
        if ((BC - L3 * np.cos(alpha)) / L2 < -1 or (BC - L3 * np.cos(alpha)) / L2 > 1):
            continue
        theta = np.arccos((BC - L3 * np.cos(alpha)) / L2)
        Ay = L3 * np.sin(alpha)
        Ax = BC - L3 * np.cos(alpha)
        Ey = L3 * np.sin(alpha) - 600 * np.sin(theta)

        for i in range(2):
            Ey_modified = Ey if i == 0 else 2 * Ay - Ey
            E = (0, Ey_modified)
            A = (Ax, Ay)
            B = (BC, 0)

            if (0 <= Ey_modified <= 500) and not check_collisions([E, A, B]):
                A_points.append(A)
                E_points.append(E)

# Convert lists to numpy arrays
A_points = np.array(A_points)
E_points = np.array(E_points)

# Plotting with obstacle
plt.figure(figsize=(10, 8))

# Plot points
plt.scatter(A_points[:, 0], A_points[:, 1], color='red', s=10, label='A: (Ax, Ay)')
plt.scatter(E_points[:, 0], E_points[:, 1], color='blue', s=10, label='E: (0, Ey)')

# Plot lines connecting points
for A, E in zip(A_points, E_points):
    plt.plot([E[0], A[0], BC], [E[1], A[1], 0], color='black', alpha=0.3)

# Highlight point B
plt.scatter([BC], [0], color='green', marker='o', s=100, label='B: (750, 0)')

# Plot the obstacle as a circle (center at (400,0), radius = 300)
obstacle_center = (400, 0)
obstacle_radius = 300
obstacle = plt.Circle(obstacle_center, obstacle_radius, color='purple', alpha=0.3, label="Obstacle")

# Add the obstacle to the plot
plt.gca().add_patch(obstacle)

# Labels and legend
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Visualization of A, E, B positions with Obstacle")
plt.legend()
plt.grid(True)
plt.axis("equal")  # Ensure aspect ratio is equal for correct visualization

# Show plot
plt.show()
