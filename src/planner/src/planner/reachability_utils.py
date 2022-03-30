# Functions for: create_motion_sensing_pZ, compute_reachable_sets_position_sensing, is_collision_free

import numpy as np
import math

from scipy.optimize import linprog
from scipy.stats.distributions import chi2
import cvxpy as cvx

from planner.probabilistic_zonotope import pZ


def initialize_reachability_analysis(x_nom0, P0):
    """
    TODO
    Initialize variables for the reachability analysis. To be run at t=0.
    """

    state_dim = x_nom0.shape[0]
    Xaug0 = pZ.linear_transform(np.concatenate(
        (np.identity(state_dim),np.zeros((state_dim,state_dim))),axis=0), 
        pZ( np.zeros((state_dim,1)), np.zeros((state_dim,0)), P0))
    Xaug0.c = np.tile(x_nom0,(2,1))

    return Xaug0


def create_motion_sensing_pZ(xnom, Q, R, m, bias_area_lims, regular_bias, different_bias):
    """
    Create motion and sensing p-zonotopes for the given nominal trajectory

    Parameters
    ----------
    xnom : np.array (4xN)
        Nominal states
    Q : np.array (4x4)
        Motion model covariance
    R : np.array (3x3)
        Sensing model covariance
    m : float 
        Desired confidence level
    bias_area_lims : 4-element list of scalars [x_min, y_min, x_max, y_max]
        Area limits for where position/range sensing bias is present: 
    regular_bias : float
        Amount of position/range sensing bias outside bias area
    different_bias : float
        Amount of position/range sensing bias inside bias area

    Returns
    -------
    WpZ : pZ object
        p-zonotope for motion uncertainty.
    VpZs : N-element list of pZ objects
        p-zonotopes for sensing uncertainty along nominal trajectory.
    Rhats : np.array (3x3xN)
        Approximate measurement covariance matrices to be used by EKF.
    """

    # Get trajectory length
    N = xnom.shape[1]
    # Get state and measurement dimensions
    state_dim = xnom.shape[0]
    measurement_dim = R.shape[0]

    # Get nominal trajectory indices within bias area
    bias_area_idx = (xnom[0,:] >= bias_area_lims[0]) * (xnom[1,:] >= bias_area_lims[1]) * (xnom[0,:] <= bias_area_lims[2]) * (xnom[1,:] <= bias_area_lims[3])
    
    # Initialize single bias array, assuming no bias in the heading measurements
    bias_array = np.ones((measurement_dim,1,1)); bias_array[-1,0,0] = 0

    # Create sensing bias array along nominal trajectory
    z_bias_w = regular_bias*np.repeat(bias_array, N, axis=2)
    z_bias_w[:,:,bias_area_idx] = different_bias*np.repeat(bias_array, bias_area_idx.sum(), axis=2)
    
    # Create sensing covariance matrices along nominal trajectory 
    z_cov = np.repeat(R[:,:,np.newaxis], N, axis=2)

    # Create prob zonotope for motion uncertainties
    WpZ = pZ(np.zeros((state_dim,1)), np.zeros((state_dim,0)), Q)

    # Create prob zonotopes for sensing uncertainties, and approximate measurement covariance matrices for EKF
    VpZs = [None]*N
    Rhats = np.zeros((measurement_dim, measurement_dim, N))
    for k in range(N):
        VpZs[k] = pZ(np.zeros((measurement_dim,1)), np.diag(z_bias_w[:,0,k]), z_cov[:,:,k])
        Rhats[:,:,k] = np.diag(((z_bias_w[:,0,k] + m*np.sqrt( np.diag(z_cov[:,:,k])))/m)**2) #use over-boudning hypothesis for the approximate measurement covariance matrix

    return WpZ, VpZs, Rhats


def compute_reachable_sets_position_sensing(xnom, unom, Xaug0, P0, Q, WpZ, VpZs, Rhats, Q_lqr, R_lqr, m, dt):
    """Compute reachable sets for unicycle model with position and heading sensing

    Parameters
    ----------
    xnom : np.array (4xN)
        Nominal states.
    unom : np.array (2xN)
        Nominal control inputs.
    Xaug0 : pZ object
        Initial reachable set.
    P0 : np.array (4x4)
        Initial state estimation covariance matrix.
    Q : np.array (4x4)
        Motion model covariance.
    WpZ : pZ object
        p-zonotope for motion uncertainty.
    VpZs : N-element list of pZ objects
        Prob zonotopes for sensing uncertainty along nominal trajectory.
    Rhats : np.array (3x3xN)
        Approximate measurement covariance matrices to be used by EKF.
    Q_lqr : np.array (4x4)
        LQR state weight matrix used to determine K.
    R_lqr : np.array (2x2)
        LQR control weight matrices used to determine K.
    m : float
        Desired confidence level.
    dt : float
        Discrete time-step.

    Returns
    -------
    Xaug : N-element list of pZ objects
        Probabilistic reachable sets.
    Zaug : N-element list of pZ objects without any covariance component
        Confidence reachable sets.
    P_all : np.array (4x4xN)
        State estimation covariance matrices along nominal trajectory. 

    """

    # The timesteps for which we need to compute the reachable sets
    N_timesteps = xnom.shape[1]

    # Get state, input and measurement dimensions
    state_dim = xnom.shape[0]; input_dim = unom.shape[0]
    measurement_dim = Rhats.shape[0]

    # Get confidence value for m-sigma in 2d using chi-square distribution
    conf_value = np.sqrt(chi2.ppf(math.erf(m/np.sqrt(2)),df=2))
    
    # Init state estimation covariance matrices, prob reach sets and confidence reach sets
    P_all = np.zeros((state_dim,state_dim,N_timesteps)); P_all[:,:,0] = np.copy(P0)
    Xaug = [None]*N_timesteps; Xaug[0] = Xaug0
    Zaug = [None]*N_timesteps; Zaug[0] = pZ.conf_zonotope(pZ( Xaug[0].c[0:2,:], Xaug[0].G[0:2,:], Xaug[0].Sigma[0:2,0:2] ), conf_value)

    # Iterate over nominal trajectory to compute reach sets
    for k in range(1,N_timesteps):

        # Get robot matrices
        A, B, C, K = generate_robot_matrices(xnom[:,[k-1]], unom[:,[k-1]], Q_lqr, R_lqr, dt)

        # Perform the EKF predict step
        P_pred = A @ P_all[:,:,k-1] @ A.T + Q

        # Perform the EKF update/correction step
        L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + Rhats[:,:,k])
        P_all[:,:,k] = P_pred - L @ C @ P_pred

        # Recursive reach coefficients for previous reach set, motion uncertainty and sensing uncertainty
        phi = np.block([[A, -B@K],[L@C@A , A - B@K - L@C@A]])
        phi_w = np.concatenate((np.identity(state_dim), L@C), axis=0)
        phi_v = np.concatenate((np.zeros((state_dim,measurement_dim)), L), axis=0)

        # Recursive reach coefficients for lagrange remainders
        phi_Lf1 = np.concatenate((np.identity(state_dim), L@C),axis=0)
        phi_Lf2 = np.concatenate((np.zeros((state_dim,state_dim)), np.identity(state_dim) - L@C),axis=0)

        # Compute prob zonotopes needed for motion model lagrange remainders
        del_s = pZ.linear_transform(
            np.block([[np.identity(state_dim), np.zeros((state_dim,state_dim))], [np.zeros((input_dim,state_dim)), -K]]), 
            pZ(np.zeros((2*state_dim,1)), Xaug[k-1].G, Xaug[k-1].Sigma))
        del_shat = pZ.linear_transform(np.block([[np.zeros((state_dim,state_dim)), np.identity(state_dim)], [np.zeros((input_dim,state_dim)), -K]]), 
            pZ(np.zeros((2*state_dim,1)), Xaug[k-1].G, Xaug[k-1].Sigma))

        # Compute motion model lagrange remainders
        Lf1 = lagrange_remainder_f( del_s, xnom[:,[k-1]], unom[:,[k-1]], m, dt)
        Lf2 = lagrange_remainder_f( del_shat, xnom[:,[k-1]], unom[:,[k-1]], m, dt)

        # Get each term for obtaining reach set for timestep k
        t2 = pZ.linear_transform(phi, pZ(np.zeros((2*state_dim,1)), Xaug[k-1].G, Xaug[k-1].Sigma))
        t3 = pZ.linear_transform(phi_w, WpZ)
        t4 = pZ.linear_transform(phi_v, VpZs[k])
        t5 = pZ.linear_transform(phi_Lf1, Lf1)
        t6 = pZ.linear_transform(phi_Lf2, Lf2)

        # Add inidividual terms to get reach set which is centered at the nominal trajectory
        Xaug[k] = pZ.minkowski_sum([t2, t3, t4, t5, t6])
        Xaug[k].c = np.tile(xnom[:,[k]],(2,1))

        # Generate confidence zonotopes for collision checks
        Zaug[k] = pZ.conf_zonotope(pZ( Xaug[k].c[0:2,:], Xaug[k].G[0:2,:], Xaug[k].Sigma[0:2,0:2]), conf_value)

    return Xaug, Zaug, P_all


def is_collision_free( Zaug, unsafeZ, collisionCheckOrder=np.inf, distCheck=False, dist_threshold=np.inf):
    """Check if confidence reachable sets are collision free w.r.t. unsafe zonotope

    Parameters
    ----------
    Zaug : N-element list of pZ objects without any covariance component
        Confidence reachable sets.
    unsafeZ : M-element list of pZ objects without any covariance component
        Sets to avoid.
    collisionCheckOrder : int
        Maximum order of reach sets in Zaug for collision checking.
    distCheck : bool
        Flag to determine whether to consider only nearby obstacles for collision checking.
    dist_threshold : float
        Distance threshold for determining nearby obstacles.
    
    Returns
    -------
    isCollisionFree : bool
        Flag indicating if the confidence reach sets Zaug are collision free.

    """

    # Get length of trajectory and number of unsafe sets
    N = len(Zaug); M = len(unsafeZ)
    
    # Init collision free flag and iterate over reach sets
    isCollisionFree = True

    for k in range(N):

        # Reduce order of reach set if specified max order is not inf
        if not np.isinf(collisionCheckOrder):
            tZ = pZ.reduce(Zaug[k], collisionCheckOrder)
        else:
            tZ = Zaug[k]

        # Iterate over unsafe sets        
        for j in range(M):
            
            # Check if unsafe set is closer than specified distance threshold
            if distCheck:
                if np.linalg.norm( tZ.c - unsafeZ[j].c ) >= dist_threshold:
                    continue

            # Create constrained zonotope for intersection between reduced reach set and unsafe set
            conZ_A = np.concatenate(( tZ.G, -unsafeZ[j].G ), axis=1)
            conZ_b = unsafeZ[j].c - tZ.c

            # Zonotopes are intersecting if constrained zonotope is not empty
            isIntersecting = not is_empty_con_zonotope(conZ_A, conZ_b)

            # Update collision free flag if reach set intersects unsafe set
            if isIntersecting:
                isCollisionFree = False
                break

        # If collision has been detected then break out of loop
        if not isCollisionFree:
            break

    return isCollisionFree

def is_empty_con_zonotope(A, b, method='scipy'):
    """Check if constrained zonotope is empty. 
    
    Used to detect intersection between reach set and unsafe set.
    Implemented by Adam Dai, Derek Knowles (http://cs229.stanford.edu/proj2021spr/report2/81976691.pdf)
    """

    # Dimension of problem
    d = A.shape[1]

    # Cost
    f_cost = np.zeros((d, 1))
    f_cost = np.concatenate((f_cost, np.eye(1)), axis=0)

    # Inequality cons
    A_ineq = np.concatenate((-np.eye(d), -np.ones((d, 1))), axis=1)
    A_ineq = np.concatenate((A_ineq, np.concatenate((np.eye(d), -np.ones((d, 1))), axis=1)), axis=0)
    b_ineq = np.zeros((2 * d, 1))

    # Equality cons
    A_eq = np.concatenate((A, np.zeros((A.shape[0], 1))), axis=1)
    b_eq = b

    if method == 'scipy':
        res = linprog(f_cost, A_ineq, b_ineq, A_eq, b_eq, (None, None))
        x = res.x
    elif method == 'cvxpy':
        # Shubh: typically faster for high dimensional problems
        x_cvx = cvx.Variable((f_cost.shape[0], 1))
        cost = np.transpose(f_cost) @ x_cvx
        constraints = [A_ineq @ x_cvx <= b_ineq, A_eq @ x_cvx == b_eq]
        problem = cvx.Problem(cvx.Minimize(cost), constraints)
        problem.solve()
        x = x_cvx.value
    else:
        raise Exception('Invalid method!')

    if x[-1] <= 1:
        return False
    return True


def lagrange_remainder_f(del_s_, xnom_, unom_, m, dt):
    """Lagrange remainder function

    Approximation for error in linearization of nonlinear motion model called by function "compute_reachable_sets_position_sensing".

    """

    state_dim = xnom_.shape[0]
    input_dim = unom_.shape[0]

    del_s_Z = pZ.conf_zonotope(del_s_, m)
    gamma = np.reshape(np.sum(np.abs(del_s_Z.G), axis=1) , (state_dim+input_dim,1))

    t1 = xnom_[2,0] - gamma[2,0]; t2 = xnom_[2,0] + gamma[2,0]

    if np.floor(t1/np.pi) != np.floor(t2/np.pi):
        cos_max = 1.0
    else:
        cos_max = np.maximum(np.abs(np.cos(t1)) , np.abs(np.cos(t2)))

    if np.floor((t1 - np.pi/2)/np.pi) != np.floor((t2 - np.pi/2)/np.pi):
        sin_max = 1.0
    else:
        sin_max = np.maximum(np.abs(np.sin(t1)) , np.abs(np.sin(t2)))

    V_max = xnom_[3,0] + gamma[3,0]

    maxJ0 = np.zeros((state_dim+input_dim,state_dim+input_dim))
    maxJ1 = np.zeros((state_dim+input_dim,state_dim+input_dim))

    maxJ0[2,2] = V_max * cos_max * dt
    maxJ0[2,3] = sin_max * dt; maxJ0[3,2] = sin_max * dt

    maxJ1[2,2] = V_max * sin_max * dt
    maxJ1[2,3] = cos_max * dt; maxJ1[3,2] = cos_max * dt

    LR = np.zeros(state_dim)
    LR[0] = 0.5 * gamma.T @ maxJ0 @ gamma
    LR[1] = 0.5 * gamma.T @ maxJ1 @ gamma
    LR[2] = 0
    LR[3] = 0

    L = pZ(np.zeros((state_dim,1)), np.zeros((state_dim,0)), np.diag(LR/m)**2)

    return L


def generate_robot_matrices(x_nom, u_nom, Q_lqr, R_lqr, dt):
    """Generate A, B, C and K matrices for robot based on given nominal state and input vector.

    Parameters
    ----------
    x_nom : np.array (4x1)
        nominal state
    u_nom : np.array (2x1)
        nominal input
    Q_lqr : np.array (4x4)
        LQR state cost weight matrix
    R_lqr: np.array (2x2)
        LQR input cost weight matrix
    dt: float
        discrete time-step

    Returns
    -------
    A : np.array (4x4)
        Linearized motion model matrix.
    B : np.array (4x2)
        Linearized control input matrix.
    C : np.array (3x4)
        Measurement matrix.
    K : np.array (2x4)
        Control feedback gain matrix.

    """
    # Get state dimension. 
    state_dim = x_nom.shape[0]
    input_dim = u_nom.shape[0]
    measurement_dim = 3  # assuming 2D position and heading measurement

    # Form linearized motion model matrix
    A = np.identity((state_dim))
    A[0,2] = -x_nom[3,0]*np.sin(x_nom[2,0])*dt
    A[0,3] = np.cos(x_nom[2,0])*dt
    A[1,2] = x_nom[3,0]*np.cos(x_nom[2,0])*dt
    A[1,3] = np.sin(x_nom[2,0])*dt

    # Form linearized control input matrix
    B = np.zeros((state_dim,input_dim))
    B[2,0] = dt; B[3,1] = dt

    # Form measurement matrix
    C = np.zeros((measurement_dim, state_dim))
    C[0,0] = 1; C[1,1] = 1; C[2,2] = 1

    # Compute control feedback gain matrix
    if np.abs(x_nom[3,0]) > 0.01:  # if there is sufficient speed for controllability
        K = dlqr_calculate(A, B, Q_lqr, R_lqr)
    else:
        K = np.array([[0, 0, 1.0, 0], 
                      [0, 0, 0, 1.0]])  # tuned from flight room tests

    return A, B, C, K


def dlqr_calculate(G, H, Q, R):
    """
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]
    Implementation from  https://github.com/python-control/python-control/issues/359#issuecomment-759423706

    How to apply the function:    
        K = dlqr_calculate(G,H,Q,R)
        K, P, E = dlqr_calculate(G,H,Q,R, return_solution_eigs=True)

    Inputs:
      G, H, Q, R  -> all numpy arrays  (simple float number not allowed)
      returnPE: define as True to return Ricatti solution and final eigenvalues

    Returns:
      K: state feedback gain
      P: Ricatti equation solution
      E: eigenvalues of (G-HK)  (closed loop z-domain poles)
      
    """
    from scipy.linalg import solve_discrete_are, inv
    P = solve_discrete_are(G, H, Q, R)  #Solução Ricatti
    K = inv(H.T@P@H + R)@H.T@P@G    #K = (B^T P B + R)^-1 B^T P A 

    return K