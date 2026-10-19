import numpy as np
from ..domain.evaluation import EvaluationGeometry

class EvaluateMap:
    def __init__(self, geometry: EvaluationGeometry, report):
        self.geometry = geometry
        self.report = report
    def execute(self, headless=False):
        self.geometry.prepare()
        # ==========================================
        # 4. CALCULATE ACADEMIC METRICS
        # ==========================================
        self.report("[4/5] Calculating Paper Metrics...")

        # A. ACCURACY (How wrong is the map?) -> Distance from Map to GT
        map_to_gt_dists = np.asarray(self.geometry.map_distances())
        mean_error = np.mean(map_to_gt_dists)
        rmse = np.sqrt(np.mean(map_to_gt_dists**2))

        # B. COMPLETENESS / COVERAGE (How much of the pipe did we see?) -> Distance from GT to Map
        gt_to_map_dists = np.asarray(self.geometry.ground_truth_distances())

        # If a GT point is within 15cm of a mapped point, we consider it "Covered"
        coverage_threshold = 0.15 
        covered_points = np.sum(gt_to_map_dists < coverage_threshold)
        coverage_percentage = (covered_points / len(gt_to_map_dists)) * 100.0

        self.report("\n=======================================")
        self.report("          BENCHMARK RESULTS")
        self.report("=======================================")
        self.report(f"Mean Error (Accuracy):  {mean_error:.4f} m")
        self.report(f"RMSE (Accuracy):        {rmse:.4f} m")
        self.report(f"Surface Coverage:       {coverage_percentage:.2f} % (at {coverage_threshold}m threshold)")
        self.report("=======================================\n")

        if headless:
            return coverage_percentage, rmse

        self.geometry.visualize(map_to_gt_dists)
