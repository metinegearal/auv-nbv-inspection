from scipy.optimize import minimize

class ScipySolver:
    def minimize(self, objective, initial, **options):
        return minimize(objective, initial, **options)
