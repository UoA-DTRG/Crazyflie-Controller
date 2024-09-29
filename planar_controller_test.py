import time
import matplotlib.pyplot as plt
import control# type: ignore
import numpy as np

DragCoef = 0.01
BeamMass = 0.01
BeamMOI = 0.1
BeamLength = 1
Xi = np.zeros(6)  # zero initial conditions

# Define the A Matrix
A = np.zeros((6, 6))
A[0, 1] = 1
A[1, 1] = -DragCoef
A[2, 3] = 1
A[3, 3] = -DragCoef
A[4, 5] = 1
A[5, 5] = -DragCoef

# Define the B Matrix
B = np.zeros((6, 3))
B[1, 0] = 1 / BeamMass
B[3, 1] = 1 / BeamMass
B[5, 2] = BeamLength / BeamMOI

# Define the C Matrix
C = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0]
])

# Define the D Matrix
D = np.zeros((3, 3))

Abar = np.block([
    [np.zeros((3, 3)), C],
    [np.zeros((6, 3)), A]
])

Bbar = np.vstack([
    np.zeros((3, 3)),
    B
])

Q = np.diag([100, 100, 20, 1, 1e-7, 1, 1e-7, 1, 1e-7])
print(Q)
R = np.diag([80, 80, 80])

# K, S, E = control.lqr(Abar, Bbar, Q, R)


# # Split the gain matrix
# Kx = K[:, :A.shape[1]]
# Kr = K[:, A.shape[1]:]

# USE THE MATLAB KX AND KR VALUES



Kx = np.array([[0.336006188846836, 0.0449000160698343, -4.56553643367304e-17, -1.81939417777330e-18, 1.71735803176753e-17, -9.97173155285121e-19],
                [3.06968847690463e-17, 5.28181698216148e-18, 0.336006188846836, 0.0449000160698342, 7.25443127949367e-16, 5.67051224398385e-17],
                [7.90360431260520e-17, -1.53072498781588e-18, -4.34980004740973e-16, -3.81930602143269e-17, 0.144960731028726, 0.00851358854038272]])

Kr = np.array([[1.11803398874990, 1.29179826320820e-16, -2.37786211159190e-16],
                [-7.25042432961745e-17, 1.11803398874989, 2.35694626584599e-15],
                [-4.96871818511696e-16, -1.90044870473925e-15, 0.499999999999999]])

print(Kx)
print(Kr)
# Initial state
x = np.zeros(6)


# Simulation parameters
dt = 0.001  # Time step
t_end = 10  # End time
time = np.arange(0, t_end, dt)

# Storage for results
x_history = []
u_history = []
ref_history = []
y_tracker = np.zeros(3)
# Simulation loop
for t in time:
    # Define reference signal with a step after 5 seconds
    if t < 5:
        reference = np.array([0, 0, 0])  # Initial reference
    else:
        reference = np.array([1, 0, 0])  # Step reference after 5 seconds
    if t == 5:
        print("5 seconds")
    
    ref_history.append(reference)
    # Compute control input
    y_tracker += Kr @ (reference - (C @ x))
    print(x)
    # u = -Kx @ x + y_tracker

    u = np.array([1, 0, 0])
    # Update state
    x = A @ x + B @ u
    print (x)
    # Store results
    x_history.append(x)
    u_history.append(u)

# Convert results to numpy arrays for easier handling
x_history = np.array(x_history)
u_history = np.array(u_history)
ref_history = np.array(ref_history)
# Print final state and control input
print("Final state:", x)
print("Final control input:", u)

# Plotting the results
plt.figure(figsize=(12, 6))

# Plot state response
plt.subplot(3, 1, 1)
plt.plot(time, x_history)
plt.title('State Response')
plt.xlabel('Time (s)')
plt.ylabel('State')
plt.legend(['x1', 'x_vel', 'y','y_vel','phi','phi_vel'])  # Adjust legend based on the number of states

# Plot control input
plt.subplot(3, 1, 2)
plt.plot(time, u_history)
plt.title('Control Input')
plt.xlabel('Time (s)')
plt.ylabel('Control Input')
plt.legend(['Fx', 'Fy', 'Mz'])  # Adjust legend based on the number of inputs

# plot ref signal
plt.subplot(3, 1, 3)
plt.plot(time, ref_history)
plt.title('Reference Signal')
plt.xlabel('Time (s)')
plt.ylabel('Reference Signal')
plt.legend(['x', 'y', 'phi'])  # Adjust legend based on the number of inputs

plt.tight_layout()
plt.show()