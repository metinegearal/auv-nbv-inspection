import optuna
from auv_benchmark.auv_benchmark import evaluate_map_accuracy
from .runner_adapter import RosTrialRunner
from .trial_policy import TrialObjective

objective = TrialObjective(RosTrialRunner(), evaluate_map_accuracy, print)

def main():
    study_name = "auv_nbv_optimization"

    # Robust storage with a 20-second timeout queue to prevent SQLite locks
    storage = optuna.storages.RDBStorage(
        url="sqlite:///optuna_auv_study.db",
        engine_kwargs={"connect_args": {"timeout": 20.0}}
    )

    study = optuna.create_study(
        study_name=study_name, 
        storage=storage, 
        directions=["maximize", "minimize"], 
        load_if_exists=True 
    )

    print(f"\nOptimization results will be saved to: sqlite:///optuna_auv_study.db")
    study.optimize(objective, n_trials=60)

    print("\n=======================================")
    print("Optimization Complete!")
    print("Best Trials on the Pareto Front:")
    for trial in study.best_trials:
        print(f"Trial {trial.number}: Coverage {trial.values[0]:.2f}%, Distance {trial.values[1]:.2f}m")
        print(f"  Params: {trial.params}")

    import pandas as pd
    df = study.trials_dataframe()
    df.to_csv("optuna_results_summary.csv")
    print("Saved summary to optuna_results_summary.csv")
if __name__ == '__main__':
    main()
