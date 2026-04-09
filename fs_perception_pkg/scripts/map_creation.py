import math


class MapCones():
    def __init__(self):
        self.cones_array = []
        self.threshold_new_cone = 1.0

    def add_or_update_cones(self, x, y):
        cone_found = False
        for cone in self.cones_array:
            d = math.sqrt((x - cone.x)**2 + (y - cone.y)**2)

            if d < self.dist_umbral:
                # Actualizar posición (Media móvil para filtrar el error de 0.2m)
                cone.x = (cone.x * 0.8) + (x * 0.2)
                cone.y = (cone.y * 0.8) + (y * 0.2)
                cone.hits += 1
                encontrado = True
                break

        if not cone_found:
            new_cone = Cone(x, y)