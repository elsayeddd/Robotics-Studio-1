#!/usr/bin/env python3
"""Seeded synthetic sensor fixtures, never Gazebo or hardware acceptance evidence."""
import argparse
import csv
import random
from pathlib import Path

HEADER = ['trial', 'scenario', 'now_s', 'stamp_s', 'smoke_ppm', 'temperature_c',
          'wind_speed_mps', 'wind_direction_rad', 'visual_stamp_s', 'visual_confidence',
          'visual_valid', 'environment_available', 'fire_present',
          'co2_ppm', 'co2_valid', 'thermal_stamp_s', 'thermal_peak_c', 'thermal_hotspot', 'thermal_valid']


def generate(path, seeds=10, seed_start=42):
    with Path(path).open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        for scenario in ('fire', 'no_fire', 'dust', 'hot_surface', 'visual_decoy', 'dropout'):
            for seed in range(seeds):
                rng = random.Random(seed + seed_start)
                for frame in range(150):
                    stamp = 1 + frame * 0.2
                    event = 6 <= stamp <= 24
                    smoke, heat, cue = 1.0, 22.0, 0.0
                    if event and scenario in ('fire', 'dropout'):
                        ramp = min(1.0, (stamp - 6) / 2.0)
                        smoke, heat, cue = 1 + 59 * ramp, 22 + 58 * ramp, 0.8
                    elif event and scenario == 'dust':
                        smoke = 60.0
                    elif event and scenario == 'hot_surface':
                        heat = 80.0
                    elif event and scenario == 'visual_decoy':
                        cue = 1.0
                    smoke = max(0, smoke + rng.gauss(0, 2))
                    heat += rng.gauss(0, 0.7)
                    # Independent CO2 noise and an image-derived IR cue fixture.
                    # These are synthetic algorithm inputs, not a rendered thermal camera trial.
                    co2 = max(0, 420 + (1000 * ramp if event and scenario in ('fire', 'dropout') else 0) + rng.gauss(0, 5))
                    peak = 100 if event and scenario in ('fire', 'dropout', 'hot_surface') else 22
                    available = not (scenario == 'dropout' and 12 <= stamp < 18)
                    writer.writerow([f'{scenario}_{seed + seed_start}', scenario, f'{stamp:.3f}',
                                     f'{stamp:.3f}', f'{smoke:.3f}', f'{heat:.3f}', 2.5,
                                     -2.35619449, f'{stamp:.3f}', cue, 1, int(available),
                                     int(event and scenario in ('fire', 'dropout')), f'{co2:.3f}', 1,
                                     f'{stamp:.3f}', peak, int(peak >= 50), 1])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--seed-start', type=int, default=42)
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error('--seeds must be positive')
    generate(args.output, args.seeds, args.seed_start)
