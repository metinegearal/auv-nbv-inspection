from auv_contracts.ports import TrialRunner

class TrialObjective:
    def __init__(self, runner: TrialRunner, evaluate, report):
        self.runner = runner
        self.evaluate = evaluate
        self.report = report
    def __call__(self, trial):
        # Sample all 6 parameters based on the requested ranges
        params_dict = {
            'min_utility_threshold': trial.suggest_float('min_utility_threshold', 0.05, 0.35),
            'nbv_standoff': trial.suggest_float('nbv_standoff', 1.0, 2.5),
            'nbv_distance_weight': trial.suggest_float('nbv_distance_weight', 0.5, 2.0),
            'nbv_rotation_weight': trial.suggest_float('nbv_rotation_weight', 0.1, 1.0),
            'nbv_collision_weight': trial.suggest_float('nbv_collision_weight', 1.0, 5.0),
            'nbv_min_raw_gain': trial.suggest_float('nbv_min_raw_gain', 0.2, 1.0),
        }

        self.report(f"\n--- Starting Trial {trial.number} ---")
        self.report(f"Params: {params_dict}")

        total_distance = self.runner.run(params_dict)
        if total_distance is None:
            return 0.0, 999.0

        coverage, rmse = self.evaluate(headless=True)

        if rmse > 0.4:
            coverage = coverage * 0.5 

        self.report(f"Trial {trial.number} Results -> Coverage: {coverage:.2f}%, Distance: {total_distance:.2f}m")
        return coverage, total_distance
