def prune_path(grid, path):
    if not path:
        return []

    pruned = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        while j > i + 1:
            if has_line_of_sight(grid, path[i], path[j]):
                break
            j -= 1

        pruned.append(path[j])
        i = j

    return pruned

# ==========================================================
# Bresenham LOS
# ==========================================================
def has_line_of_sight(grid, p1, p2):
    x0, y0 = p1
    x1, y1 = p2

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1

    err = dx - dy

    while True:
        if grid[y0, x0] == 1:
            return False

        if (x0, y0) == (x1, y1):
            return True

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy
