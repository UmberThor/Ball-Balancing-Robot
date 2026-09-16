# Inverse kinematics: plate tilt in, motor angles out. See the matlab.
# Pure numpy, so this module can be run off the robot.

import numpy as np

# INVERSE KINEMATICS CONSTANTS
R_base = 28.65                           # radius of the base
R_plane = 84.00                          # radius of the plate
L1 = 80.0                                # length of link 1, the upper one (Pi -> Bi)
L2 = 80.0                                # length of link 2, the lower one (Mi -> Pi)
H_NOM = 120.0                            # height of the plate center at the nominal pose
base_c = np.array([0.0, 0.0, 0.0])       # coordinates of the base center
M1 = np.array([R_base*np.cos(np.pi),     R_base*np.sin(np.pi),     0.0]) # coordinates of motor 1
M2 = np.array([R_base*np.cos(5*np.pi/3), R_base*np.sin(5*np.pi/3), 0.0]) # coordinates of motor 2
M3 = np.array([R_base*np.cos(np.pi/3),   R_base*np.sin(np.pi/3),   0.0]) # coordinates of motor 3


# solve_P: given the coordinates of the motors (M) and of the ball joints (B),
# and the scalar factor that determines on which plane the pin joint must lie on (E),
# find the coordinates of the three pin joints
def solve_P(M, B, E, sign):
    A = (L1**2 - L2**2 + (M[0]**2 - B[0]**2 + M[1]**2 - B[1]**2 + M[2]**2 - B[2]**2))/(2*(M[2] - B[2]))
    Bc = (B[0] - M[0])/(M[2] - B[2])
    Cc = (B[1] - M[1])/(M[2] - B[2])
    a_coef = 1 + E**2 + (Bc + Cc*E)**2
    b_coef = -2*B[0] - 2*E*B[1] + 2*(Bc + Cc*E)*(A - B[2])
    c_coef = B[0]**2 + B[1]**2 + (A - B[2])**2 - L1**2
    Px = (-b_coef + sign*np.sqrt(b_coef**2 - 4*a_coef*c_coef))/(2*a_coef)
    return np.array([Px, E*Px, A + (Bc + Cc*E)*Px])


# solve: given the control action u and the height of the plate, find the three
# motor angles in degrees. Returns None if the requested pose is not reachable.
def solve(u, h):
    n = np.array([u[0], u[1], 1.0])
    n = n / np.linalg.norm(n)
    B1 = np.array([
        -n[2]*R_plane/np.sqrt(n[0]**2 + n[2]**2),
        0.0,
        h + n[0]*R_plane/np.sqrt(n[0]**2 + n[2]**2)
    ])
    B2 = np.array([
        n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2),
        -np.sqrt(3)*n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2),
        h + R_plane*(n[1]*np.sqrt(3) - n[0])/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) - n[0])**2)
    ])
    B3 = np.array([
        n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2),
        np.sqrt(3)*n[2]*R_plane/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2),
        h - R_plane*(n[1]*np.sqrt(3) + n[0])/np.sqrt(4*n[2]**2 + (n[1]*np.sqrt(3) + n[0])**2)
    ])
    dist1 = np.linalg.norm(B1-M1)
    dist2 = np.linalg.norm(B2-M2)
    dist3 = np.linalg.norm(B3-M3)
    ok = (abs(L1-L2) < dist1 < L1+L2 and abs(L1-L2) < dist2 < L1+L2 and abs(L1-L2) < dist3 < L1+L2)
    if not ok:
        return None
    P1 = solve_P(M1, B1, 0.0, -1)
    P2 = solve_P(M2, B2, -np.sqrt(3), +1)
    P3 = solve_P(M3, B3, np.sqrt(3), +1)
    q1 = np.arctan(np.sqrt((P1[1]-M1[1])**2 + (P1[0]-M1[0])**2)/abs(P1[2]-M1[2]))
    q2 = np.arctan(np.sqrt((P2[1]-M2[1])**2 + (P2[0]-M2[0])**2)/abs(P2[2]-M2[2]))
    q3 = np.arctan(np.sqrt((P3[1]-M3[1])**2 + (P3[0]-M3[0])**2)/abs(P3[2]-M3[2]))
    return np.degrees([q1, q2, q3])
