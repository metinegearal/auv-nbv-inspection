import argparse

from .midrun_logger import generate_results_plot


def main(args=None):
    parser = argparse.ArgumentParser(description='Plot mid-run NBV benchmark results')
    parser.add_argument('--results-dir', default='results')
    parser.add_argument('--trajectory-prefix', default='benchmark')
    parser.add_argument('--planner-prefix', default='planner')
    parser.add_argument('--hide-reference-objects', action='store_true',
                        help='Hide the reference rectangles and pipes')
    options = parser.parse_args(args)
    output = generate_results_plot(
        options.results_dir,
        options.trajectory_prefix,
        options.planner_prefix,
        show_reference_objects=not options.hide_reference_objects)
    if output is None:
        raise SystemExit('No trajectory data or matplotlib installation found')
    print(output)


if __name__ == '__main__':
    main()
