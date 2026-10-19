"""CSV, text, and response plots for measured controller waypoint episodes."""
import csv
import math


METRIC_FIELDS = (
    'episode_id', 'end_reason', 'valid_step',
    'start_x_m', 'start_y_m', 'start_z_m', 'target_x_m', 'target_y_m', 'target_z_m',
    'initial_distance_m', 'band_m', 'duration_s', 'rise_time_s', 't63_s',
    'settling_time_s', 'in_band_for_s', 'settling_dwell_met', 'overshoot_pct',
    'peak_time_s', 'final_error_m', 'iae_m_s', 'ise_m2_s', 'sample_count', 'retained_samples',
)
SAMPLE_FIELDS = ('episode_id', 'elapsed_s', 'error_m', 'progress',
                 'error_x_m', 'error_y_m', 'error_z_m')


def write_motion_response(results_path, snapshot, diagnostics):
    """Save all episodes; plot the last twelve valid position steps."""
    with (results_path / 'motion_response_metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        writer.writerows(snapshot.response_log)
    with (results_path / 'motion_response_samples.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(SAMPLE_FIELDS)
        writer.writerows(snapshot.response_samples)
    with (results_path / 'motion_response_summary.txt').open('w') as stream:
        stream.write('Measured waypoint position responses (monotonic callback receipt time)\n')
        stream.write('Rise: 10–90%; t63: 63.2% progress, not a fitted time constant.\n')
        stream.write('Progress/overshoot project onto the initial-to-target line.\n')
        stream.write('Settling uses full 3D error; all settling times are provisional.\n')
        stream.write('Missing crossings are N/A; target changes end an observation window.\n')
        stream.write('IAE: m*s; ISE: m^2*s. No plant poles or damping are inferred.\n\n')
        if not snapshot.response_log:
            stream.write('No waypoint/odometry response episodes observed.\n')
        for record in snapshot.response_log:
            stream.write(f'Episode {record["episode_id"]} ({record["end_reason"]})\n')
            for name in METRIC_FIELDS[2:]:
                value = record[name]
                rendered = ('N/A' if value is None else
                            f'{value:.6g}' if isinstance(value, float) else str(value))
                stream.write(f'  {name}: {rendered}\n')
    _plot_responses(results_path, snapshot, diagnostics)
    diagnostics.info(f'Saved motion response metrics, samples, and summary to {results_path}')


def _plot_responses(results_path, snapshot, diagnostics):
    records = [record for record in snapshot.response_log if record['valid_step']][-12:]
    if not records:
        return
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        diagnostics.warning('Matplotlib unavailable; skipping motion response plot')
        return
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    progress_axis, error_axis, times_axis, overshoot_axis = axes.flat
    samples = {record['episode_id']: [] for record in records}
    for sample in snapshot.response_samples:
        if sample[0] in samples:
            samples[sample[0]].append(sample)
    for record in records:
        rows = samples[record['episode_id']]
        label = f'#{record["episode_id"]}'
        times = [row[1] for row in rows]
        progress_axis.plot(times, [100 * row[3] for row in rows], label=label)
        line, = error_axis.plot(
            times, [100 * row[2] / record['initial_distance_m'] for row in rows], label=label)
        error_axis.axhline(100 * record['band_m'] / record['initial_distance_m'],
                           color=line.get_color(), linestyle=':', alpha=0.35)
    for threshold in (10, 100 * (1 - math.exp(-1)), 90, 100):
        progress_axis.axhline(threshold, color='gray', linestyle=':', alpha=0.4)
    progress_axis.set(title='Position response along initial target direction',
                      xlabel='Elapsed receipt time (s)', ylabel='Projected progress (%)')
    error_axis.set(title='Full 3D error (dotted lines: settling bands)',
                   xlabel='Elapsed receipt time (s)', ylabel='Error / initial distance (%)')
    indices = list(range(len(records)))
    series = ((-0.25, 'rise_time_s', 'Rise 10–90%'),
              (0., 't63_s', 't63'),
              (0.25, 'settling_time_s', 'Settling (provisional)'))
    for offset, field, label in series:
        # Unreached crossings are omitted, never rendered as zero response time.
        positions, values = [], []
        for index, record in enumerate(records):
            if record[field] is not None:
                positions.append(index + offset)
                values.append(record[field])
        times_axis.bar(positions, values, width=0.25, label=label)
    labels = [f'#{record["episode_id"]}' for record in records]
    times_axis.set(title='Response times (missing bars: N/A)', ylabel='Time (s)')
    overshoot_axis.bar(indices, [record['overshoot_pct'] for record in records], color='#ea580c')
    overshoot_axis.set(title='Projected position overshoot', ylabel='Overshoot (%)')
    for axis in (times_axis, overshoot_axis):
        axis.set_xticks(indices, labels)
        axis.set_xlabel('Waypoint response episode')
    for axis in axes.flat:
        axis.grid(True, alpha=0.2)
    for axis in (progress_axis, error_axis, times_axis):
        axis.legend(fontsize='small')
    figure.suptitle('Measured motion control responses — last 12 valid steps')
    figure.savefig(results_path / 'motion_control_response.png', dpi=160)
    plt.close(figure)
