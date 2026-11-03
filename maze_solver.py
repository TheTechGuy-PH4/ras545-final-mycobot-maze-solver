import cv2
import numpy as np
import heapq

def detect_maze_path_points(image_path, real_width_mm=380, real_height_mm=225):
    """
    Solves the maze and returns 10 (x, y) points in mm: start, 8 intermediate points, goal.
    """
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image at path '{image_path}' not found or cannot be loaded.")

    image_height_px, image_width_px = image.shape[:2]

    # Calculate pixel-to-mm scale
    x_scale = real_width_mm / image_width_px
    y_scale = real_height_mm / image_height_px

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Red marker detection
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    # Green marker detection
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([90, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Find centroids
    cnts_r = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts_g = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if not cnts_r or not cnts_g:
        raise Exception("Start or Goal not detected.")

    c_r = max(cnts_r, key=cv2.contourArea)
    c_g = max(cnts_g, key=cv2.contourArea)
    M_r = cv2.moments(c_r)
    M_g = cv2.moments(c_g)
    sx, sy = int(M_r['m10']/M_r['m00']), int(M_r['m01']/M_r['m00'])
    gx, gy = int(M_g['m10']/M_g['m00']), int(M_g['m01']/M_g['m00'])
    start = (sy, sx)
    goal = (gy, gx)

    # Walls extraction
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, walls = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    walls[mask_red > 0] = 0
    walls[mask_green > 0] = 0

    # Inflate walls
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (81, 81))  # disk radius ~30
    walls = cv2.dilate(walls, kernel, iterations=1)

    # Carve safe zones
    for center in [(sy, sx), (gy, gx)]:
        y, x = center
        cv2.circle(walls, (x, y), 80, 0, -1)  # carve circle radius 80 pixels

    occ = (walls > 0).astype(np.uint8)

    # Distance Transform
    free = (occ == 0).astype(np.uint8) * 255
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    max_dist = dist.max()
    cost_map = 1.0 + (1.0 - dist / max_dist)

    # Dijkstra's Algorithm
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    rows, cols = image.shape[:2]
    INF = float('inf')
    dists = np.full((rows, cols), INF)
    parent = np.full((rows, cols, 2), -1, dtype=int)
    heap = [(0.0, start)]
    dists[start] = 0.0

    found = False
    while heap:
        current_cost, (r, c) = heapq.heappop(heap)
        if (r, c) == goal:
            found = True
            break
        if current_cost > dists[r, c]:
            continue
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and occ[nr, nc] == 0:
                new_cost = current_cost + cost_map[nr, nc]
                if new_cost < dists[nr, nc]:
                    dists[nr, nc] = new_cost
                    parent[nr, nc] = (r, c)
                    heapq.heappush(heap, (new_cost, (nr, nc)))

    if not found:
        raise Exception("No path found")

    # Reconstruct path
    path = []
    node = goal
    while node != (-1, -1):
        path.append(node)
        node = tuple(parent[node]) if parent[node][0] != -1 else (-1, -1)
    path.reverse()
    pts = [(c, r) for (r, c) in path]  # to (x, y)

    # Interpolate 10 points
    cumulative_dists = [0.0]
    for i in range(1, len(pts)):
        x0, y0 = pts[i-1]
        x1, y1 = pts[i]
        cumulative_dists.append(cumulative_dists[-1] + np.hypot(x1-x0, y1-y0))

    total_dist = cumulative_dists[-1]
    step_size = total_dist / 9
    targets = [step_size * i for i in range(10)]

    inter_pts = []
    current_segment = 0
    for target in targets:
        while current_segment < len(pts) - 1:
            seg_start = cumulative_dists[current_segment]
            seg_end = cumulative_dists[current_segment + 1]
            if seg_start <= target <= seg_end:
                ratio = (target - seg_start) / (seg_end - seg_start)
                x0, y0 = pts[current_segment]
                x1, y1 = pts[current_segment + 1]
                xi = int(round(x0 + ratio * (x1 - x0)))
                yi = int(round(y0 + ratio * (y1 - y0)))
                inter_pts.append((xi, yi))
                break
            current_segment += 1

    # Convert to mm
    points_mm = [(x * x_scale, y * y_scale) for x, y in inter_pts]

    return points_mm

if __name__ == "__main__":
    image_path = 'unsolved_maze2.png'
    points_mm = detect_maze_path_points(image_path)
    print("10 line points in mm:", points_mm)

    # Visualization
    image = cv2.imread(image_path)
    if image is not None:
        for (x_mm, y_mm) in points_mm:
            x_px = int(round(x_mm * image.shape[1] / 380))
            y_px = int(round(y_mm * image.shape[0] / 225))
            cv2.circle(image, (x_px, y_px), 5, (255, 255, 0), -1)
        cv2.imshow('Maze Points', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
