import heapq
import numpy as np

def astar_3d(start_pos, goal_pos, occ_set, res, max_steps=50000):
    """
    A lightning-fast 3D A* pathfinder using a 6-way voxel grid.
    """
    start_idx = tuple(np.floor(start_pos / res).astype(int))
    goal_idx = tuple(np.floor(goal_pos / res).astype(int))

    if start_idx in occ_set or goal_idx in occ_set:
        print("A* Error: Start or Goal is inside an obstacle!")
        return [] 

    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}
    g_score = {start_idx: 0}
    
    # Manhattan distance heuristic is strictly optimal for 6-way movement
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1]) + abs(a[2]-b[2])
        
    directions = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]
    
    steps = 0
    while open_set and steps < max_steps:
        steps += 1
        _, current = heapq.heappop(open_set)

        if current == goal_idx:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            # Convert indices back to metric coordinates
            return [(np.array(idx) + 0.5) * res for idx in path]

        for d in directions:
            neighbor = (current[0]+d[0], current[1]+d[1], current[2]+d[2])

            if neighbor in occ_set:
                continue

            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal_idx)
                heapq.heappush(open_set, (f, neighbor))

    print("A* failed to find a path (hit step limit or target unreachable).")
    return []

# def check_line_of_sight(p_start, p_end, occ_set, res):
#     """
#     Simulates a fast raycast from the camera to the target.
#     Returns True if the line is clear, False if it hits an Occupied voxel.
#     """
#     ray_vec = p_end - p_start
#     dist = np.linalg.norm(ray_vec)
#     steps = int(dist / res)
    
#     if steps <= 1: return True
    
#     # Walk along the line from the viewpoint to the frontier
#     # We stop just BEFORE the end, because the end IS the occupied frontier!
#     for i in range(1, steps):
#         p = p_start + (ray_vec * (i / steps))
#         idx = tuple(np.floor(p / res).astype(int))
        
#         if idx in occ_set:
#             return False  # Collision!
#     return True

def check_line_of_sight(p_start, p_end, occ_set, res, gap_multiplier=4):
    """
    Simulates a 'thick' raycast by sweeping a sphere along the line of sight.
    Ensures the camera has a realistic, wide-enough viewing corridor.
    """
    ray_vec = p_end - p_start
    dist = np.linalg.norm(ray_vec)
    steps = int(dist / res)
    
    # Calculate radius in voxels (A 4x gap means a radius of 2 voxels)
    radius = gap_multiplier // 2
    
    # If the camera is extremely close, assume clear
    if steps <= radius + 1: 
        return True
    
    # Precompute the spherical offsets ONCE so this remains blazingly fast
    offsets = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                # Only keep offsets that form a sphere, not a cube
                if dx**2 + dy**2 + dz**2 <= radius**2:
                    offsets.append((dx, dy, dz))
                    
    # Walk along the line
    # IMPORTANT: We stop `radius` steps early! Otherwise, the fat ray 
    # will collide with the target object itself as it touches the frontier.
    for i in range(1, steps - radius):
        p = p_start + (ray_vec * (i / steps))
        base_idx = np.floor(p / res).astype(int)
        
        # Check the "thick" corridor for collisions
        for dx, dy, dz in offsets:
            idx = (base_idx[0] + dx, base_idx[1] + dy, base_idx[2] + dz)
            
            if idx in occ_set:
                return False  # Collision in the gap!
                
    return True