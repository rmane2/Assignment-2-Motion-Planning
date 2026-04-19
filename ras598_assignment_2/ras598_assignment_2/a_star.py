import heapq
import math


def astar(grid, start, goal):
    width  = grid.shape[1]
    height = grid.shape[0]

    def in_bounds(c, r):
        return 0 <= c < width and 0 <= r < height

    def is_free(c, r):
        return grid[r, c] == 0

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # 8-connected motion
    neighbors = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (-1, -1), (1, -1), (-1, 1)
    ]

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for dc, dr in neighbors:
            nc = current[0] + dc
            nr = current[1] + dr

            if not in_bounds(nc, nr) or not is_free(nc, nr):
                continue

            # diagonal cost
            cost = math.hypot(dc, dr)
            tentative_g = g_score[current] + cost

            neighbor = (nc, nr)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
                came_from[neighbor] = current

    return []  # no path


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
