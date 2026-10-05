"""Version 1 of the existing human-readable mission result protocol.

The filename and text are retained for existing tools. Both producer and consumer
use this module so wording changes cannot silently diverge between components.
"""
SCHEMA_VERSION = 1
MISSION_STATS_FILENAME = 'optuna_mission_stats.txt'
DISTANCE_LABEL = 'Total Distance Traveled:'


def distance_record(distance):
    return f'{DISTANCE_LABEL} {distance:.2f} meters\n'


def distance_from_records(records):
    distance = 999.0
    for line in records:
        if DISTANCE_LABEL in line:
            distance = float(line.split(':')[1].replace('meters', '').strip())
    return distance
