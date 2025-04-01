import numpy as np

def force_att(p1, p2, charge):
    """
    Calculate force vector using vectorized operations in 3D
    Parameters:
    p1, p2 (np.array): 3D points as numpy arrays [x, y, z]
    charge (float): Charge value for force calculation (positive for attraction, negative for repulsion)
    Returns:
    np.array: 3D force vector
    """
    # Calculate displacement vector (now in 3D)
    dist_vec = p2 - p1
    # Calculate magnitude of distance
    dist_mag = np.array([np.linalg.norm(dist_vec)])
    # Calculate force magnitude
    force_mag = 50000 * charge * (np.exp(dist_mag) - 1)
    # Calculate normalized direction vector and scale by force magnitude
    return force_mag * (dist_vec / dist_mag), dist_mag

def force_rep(p1, p2, dist, charge):
    """
    Calculate force vector using vectorized operations in 3D
    Parameters:
    p1, p2 (np.array): 3D points as numpy arrays [x, y, z]
    charge (float): Charge value for force calculation (positive for attraction, negative for repulsion)
    Returns:
    np.array: 3D force vector
    """
    # Calculate displacement vector (now in 3D)
    dist_vec = p2 - p1
    # Calculate magnitude of distance
    dist_mag = np.array([dist])
    # Calculate force magnitude - reduced constant and adjusted power
    force_mag = 3000 * charge / (dist_mag ** 2.3)
    #force_mag = 100000 * charge * np.exp(-(dist_mag))
    # Calculate normalized direction vector and scale by force magnitude
    return force_mag * (dist_vec / dist_mag), dist_mag

def force_home(p1, p2, dist, charge):
    """
    Calculate force vector using vectorized operations in 3D
    Parameters:
    p1, p2 (np.array): 3D points as numpy arrays [x, y, z]
    charge (float): Charge value for force calculation (positive for attraction, negative for repulsion)
    Returns:
    np.array: 3D force vector
    """
    # Calculate displacement vector (now in 3D)
    dist_vec = p2 - p1
    # Calculate magnitude of distance
    dist_mag = np.array([np.linalg.norm(dist_vec)])
    # Calculate force magnitude
    force_mag = 600000 * charge * (np.exp(dist_mag) - 1)
    # Calculate normalized direction vector and scale by force magnitude
    return force_mag * (dist_vec / dist_mag), dist_mag

def force_resultant(f1, f2, velocity, damping_coeff=5.0):
    """
    Calculate resultant force with damping
    Parameters:
    f1, f2 (np.array): Force vectors to combine
    velocity (np.array): Current velocity vector
    damping_coeff (float): Damping coefficient (higher values = more damping)
    """
    # Calculate base resultant force
    f = f1 + f2
    
    # Calculate damping force (opposing motion)
    damping_force = -damping_coeff * velocity
    
    # Add damping to resultant force
    f_damped = f + damping_force
    
    return f_damped

def apf(agent_p, goal_p, obs_p, home_p, dist, velocity, charge_goal=1, charge_ob=1, is_gripping=False, is_targeting=False, has_gripped=False):
    """
    Artificial Potential Field with damping
    Parameters:
    agent_p, goal_p, obs_p (np.array): 3D positions
    dist (float): Distance value
    velocity (np.array): Current velocity of the agent
    charge_goal, charge_ob (float): Charge values for goal and obstacle
    """
    # Convert inputs to 3D numpy arrays if not already
    points = np.array([agent_p, goal_p, obs_p])
    agent_p, goal_p, obs_p = points
    
    # Calculate attractive and repulsive forces
    f_goal, dist_goal = force_att(agent_p, goal_p, charge_goal)
    f_ob, dist_obs = force_rep(agent_p, obs_p, dist, -charge_ob)
    dist_h = np.linalg.norm(agent_p - home_p)
    f_home, dist_home = force_home(agent_p, home_p, dist_h, charge_ob)
    #print(f_home)
    
    # Calculate resultant force with damping
    f_final = force_resultant(f_goal, f_ob + f_home, velocity, damping_coeff=100.0) / 4
    
    # Handle z-axis constraints
    if(is_targeting):
        if 0.09 <= agent_p[2] <= 0.1:
            f_final[2] = 0
        elif agent_p[2] < 0.09:
            f_final[2] = 10
    if(is_gripping):
        if -0.005 <= agent_p[2] <= -0.0025:
            f_final[2] = 0
        elif agent_p[2] < -0.005:
            f_final[2] = 1

    if(has_gripped):
        if(agent_p[2] < 0.15):
            f_final[2] += 100
            
    return f_final, dist_goal, [f_goal, f_ob]

# Example usage
# start_p = np.array([0, 0, 10])          # Starting point
# end_p = np.array([-10, 10, 10])         # Goal point
# ob_p = np.array([-10, -10, 10])        # Obstacle point
# f_final, dist_goal = apf(start_p,end_p,ob_p)
# print(f_final,dist_goal)
