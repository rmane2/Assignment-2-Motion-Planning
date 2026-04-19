import numpy as np
from PIL import Image
import math


class GridMap:
    def __init__(self, image_path, cell_resolution=0.2, inflation_radius_m=0.6):
        self.cell_resolution = cell_resolution
        self.inflation_radius_m = inflation_radius_m

        # Load image (grayscale)
        img = Image.open(image_path).convert('L')
        self.img = np.array(img)

        # Occupancy: 1 = obstacle, 0 = free
        self.grid = (self.img < 180).astype(np.uint8) #250 

        self.height, self.width = self.grid.shape

        # Inflate obstacles
        self._inflate_obstacles()

        # Map origin (based on your YAML)
        self.origin_x = -8.0
        self.origin_y = -8.0

    # ==========================================================
    # Inflation
    # ==========================================================
    def _inflate_obstacles(self):
        radius_cells = int(self.inflation_radius_m / self.cell_resolution)
        inflated = np.copy(self.grid)

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r, c] == 1:
                    for dr in range(-radius_cells, radius_cells + 1):
                        for dc in range(-radius_cells, radius_cells + 1):
                            nr = r + dr
                            nc = c + dc

                            if 0 <= nr < self.height and 0 <= nc < self.width:
                                if math.hypot(dr, dc) <= radius_cells:
                                    inflated[nr, nc] = 1

        self.grid = inflated

    # ==========================================================
    # Coordinate conversions
    # ==========================================================
    def world_to_cell(self, x, y):
        c = int((x - self.origin_x) / self.cell_resolution)
        r = int((y - self.origin_y) / self.cell_resolution)


        # 🔥 Flip Y (VERY IMPORTANT)
        r = self.height - r - 1

        return (c, r)

    def cell_to_world(self, c, r):
        r = self.height - r - 1
        x = c * self.cell_resolution + self.origin_x
        y = r * self.cell_resolution + self.origin_y
        return (x, y)

    # ==========================================================
    # Checks
    # ==========================================================
    def is_free_cell(self, c, r):
        if 0 <= r < self.height and 0 <= c < self.width:
            return self.grid[r, c] == 0
        return False

    def find_nearest_free(self, c, r, max_radius=50):
        """Search outward to find nearest free cell."""
        if self.is_free_cell(c, r):
            return (c, r)

        for radius in range(1, max_radius):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    nc = c + dc
                    nr = r + dr

                    if self.is_free_cell(nc, nr):
                        return (nc, nr)

        return None  # no free cell found