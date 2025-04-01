import numpy as np
import matplotlib.pyplot as plt

def ensure_numpy_array(p):
    """Helper function to ensure input is numpy array"""
    return np.array(p, dtype=float)

def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    p1 = ensure_numpy_array(p1)
    p2 = ensure_numpy_array(p2)
    return np.linalg.norm(p2 - p1)

def dist_4(i_p, p1, p2, p3, p4):
    """Calculate minimum distance from point to two line segments"""
    di1 = min(distance(i_p, p1), distance(i_p, p2))
    di2 = min(distance(i_p, p3), distance(i_p, p4))
    return min(di1, di2)

def dist_2(i_p, p1, p2):
    """Calculate minimum distance from point to line segment"""
    return min(distance(i_p, p1), distance(i_p, p2))

def calculate_configuration_vectors(p1, p2, p3, p4, epsilon=1e-10):
    """Calculate configuration vectors for line segments"""
    p1 = ensure_numpy_array(p1)
    p2 = ensure_numpy_array(p2)
    p3 = ensure_numpy_array(p3)
    p4 = ensure_numpy_array(p4)
    
    d1 = p2 - p1
    d2 = p4 - p3
    r = p3 - p1
    
    v = np.cross(d1, d2)
    v_norm = np.linalg.norm(v)
    
    n = v / v_norm if v_norm > epsilon else None
    
    return {
        'direction_1': d1,
        'direction_2': d2,
        'cross_product': v,
        'connecting_vector': r,
        'normal_vector': n,
        'cross_product_magnitude': v_norm
    }

def classify_configuration(p1, p2, p3, p4, epsilon=1e-10):
    """Classify line segment configuration"""
    vectors = calculate_configuration_vectors(p1, p2, p3, p4, epsilon)
    v_norm = vectors['cross_product_magnitude']
    
    if v_norm < epsilon:
        return 'P'
        
    r = vectors['connecting_vector']
    v = vectors['cross_product']
    dist_vect = np.cross(v, r)
    
    if np.linalg.norm(dist_vect) < epsilon:
        return 'I'
    
    return 'S'

def point_in_segment(point, seg_start, seg_end, epsilon=1e-10):
    """Check if point lies on line segment"""
    point = ensure_numpy_array(point)
    seg_start = ensure_numpy_array(seg_start)
    seg_end = ensure_numpy_array(seg_end)
    
    d = seg_end - seg_start
    s = point - seg_start
    
    mag_d = np.linalg.norm(d)
    if mag_d < epsilon:  # Degenerate segment
        return 1 if np.linalg.norm(s) < epsilon else 0
        
    # Normalized dot product gives parameter t
    t = np.dot(s, d) / (mag_d * mag_d)
    
    # Check if point projects onto segment
    return 1 if -epsilon <= t <= 1 + epsilon else 0

def skew_points(p1, p2, p3, p4, epsilon=1e-10):
    """Calculate closest points between skew lines"""
    p1 = ensure_numpy_array(p1)
    p2 = ensure_numpy_array(p2)
    p3 = ensure_numpy_array(p3)
    p4 = ensure_numpy_array(p4)

    d1 = p2 - p1
    d2 = p4 - p3
    r = p3 - p1

    v = np.cross(d1, d2)
    v_norm = np.linalg.norm(v)

    if v_norm < epsilon:  # Parallel lines
        return p1, p3

    n = v / v_norm
    A = np.vstack([d1, -d2, n]).T
    
    try:
        params = np.linalg.solve(A, r)
        l1p = p1 + params[0] * d1
        l2p = p3 + params[1] * d2
        return l1p, l2p
    except np.linalg.LinAlgError:
        return p1, p3

def distance_between_parallel_lines(p1, p2, p3, p4, epsilon=1e-10, safety_dist=0.1):
    """
    Calculate distance between parallel line segments
    Returns: (distance, collision_flag)
    """
    p1 = ensure_numpy_array(p1)
    p2 = ensure_numpy_array(p2)
    p3 = ensure_numpy_array(p3)
    p4 = ensure_numpy_array(p4)
    
    d1 = p2 - p1
    d1_norm = np.linalg.norm(d1)
    
    # Handle degenerate case where first segment is a point
    if d1_norm < epsilon:
        return distance(p1, p3), 0
        
    d1_unit = d1 / d1_norm
    r = p3 - p1
    
    # Project endpoints of second segment onto direction of first segment
    t0 = np.dot(p3 - p1, d1_unit)
    t1 = np.dot(p4 - p1, d1_unit)
    
    # Find overlap region
    tmin = max(0, min(t0, t1))
    tmax = min(d1_norm, max(t0, t1))
    
    if tmin > tmax:  # No overlap
        # Calculate distances to all endpoints
        distances = [
            distance(p1, p3),
            distance(p1, p4),
            distance(p2, p3),
            distance(p2, p4)
        ]
        min_dist = min(distances)
        return min_dist, 1 if min_dist <= safety_dist else 0
    
    # Calculate perpendicular distance component
    proj = np.dot(r, d1_unit)
    perp_vector = r - proj * d1_unit
    perp_dist = np.linalg.norm(perp_vector)
    
    return perp_dist, 1 if perp_dist <= safety_dist else 0

def detect_coll(p1, p2, p3, p4, epsilon=1e-10):
    """Detect collision between two line segments"""
    # Convert all points to numpy arrays at the start
    p1, p2, p3, p4 = map(ensure_numpy_array, [p1, p2, p3, p4])
    
    case = classify_configuration(p1, p2, p3, p4, epsilon)
    
    if case == 'P':
        dist, _ = distance_between_parallel_lines(p1, p2, p3, p4, epsilon)
        return dist
        
    elif case == 'I':
        # For intersecting lines, find intersection point
        d1 = p2 - p1
        d2 = p4 - p3
        try:
            A = np.vstack([d1, -d2]).T
            b = p3 - p1
            t, s = np.linalg.solve(A, b)
            int_point = p1 + t * d1
            
            if point_in_segment(int_point, p1, p2) and point_in_segment(int_point, p3, p4):
                return 0
        except np.linalg.LinAlgError:
            pass
        
        return min(dist_2(p1, p3, p4), dist_2(p2, p3, p4))
            
    else:  # Skew case
        l1p, l2p = skew_points(p1, p2, p3, p4, epsilon)
        min_dist = distance(l1p, l2p)
        
        if point_in_segment(l1p, p1, p2) and point_in_segment(l2p, p3, p4):
            return min_dist
        
        return min(dist_2(p1, p3, p4), dist_2(p2, p3, p4))

def create_collision_matrix(segments, epsilon=1e-10):
    """Create collision matrix for multiple segments"""
    n = len(segments)
    C = np.zeros((n, n))
    
    for i in range(n):
        for j in range(i+1, n):
            p1, p2 = segments[i]
            p3, p4 = segments[j]
            C[i,j] = detect_coll(p1, p2, p3, p4, epsilon)
            C[j,i] = C[i,j]
    
    return C

def check_global_collision(C, safety_dist):
    """Check for collisions in the collision matrix"""
    collision_pairs = np.argwhere(C <= safety_dist)
    collision_pairs = [(i,j) for i,j in collision_pairs if i < j]
    return len(collision_pairs) > 0, collision_pairs

# # # Global variables
# global safety_dist
# safety_dist = 0.1  # meters
# p1 = [1, 2, 3]
# p2 = [2, 3, 4]
# p3 = [5, 6, 7]
# p4 = [6, 7, 8]
# dist = detect_coll(p1, p2, p3, p4)
# print(f"Distance: {dist}")

# if __name__ == "__main__":
#     # Test segments
#     segments = [
#         ([1, 2, 3], [2, 3, 4]),
#         ([5, 6, 7], [6, 7, 8])
#     ]

#     # Check for collisions
#     C = create_collision_matrix(segments)
#     has_collision, collision_pairs = check_global_collision(C, safety_dist)

#     if has_collision:
#         print("Collisions detected between segments:", collision_pairs)
#     else:
#         print("No collisions detected")
