#!/usr/bin/env python3
"""Compare candidate settings on training CSV; verify separately on held-out trials."""
import argparse
import csv
import io
import itertools
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('training_csv')
    parser.add_argument('--evaluator', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    results = []
    for alpha, smoke, heat in itertools.product([0.15, 0.25, 0.4], [10.0, 12.0, 15.0], [30.0, 32.0, 35.0]):
        command = [args.evaluator, args.training_csv, '--alpha', str(alpha), '--smoke-enter', str(smoke), '--heat-enter', str(heat)]
        rows = list(csv.DictReader(io.StringIO(subprocess.check_output(command, text=True))))
        tp, fp, fn = (sum(int(row[key]) for row in rows) for key in ('tp', 'fp', 'fn'))
        # False confirmations are weighted twice as heavily as missed positive frames.
        cost = 4 * fp + 2 * fn
        results.append(dict(alpha=alpha, smoke_enter=smoke, heat_enter=heat, tp=tp, fp=fp, fn=fn, cost=cost))
    results.sort(key=lambda row: (row['cost'], row['alpha']))
    with open(args.output, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print('Lowest training cost:', results[0])
    print('Evaluate this candidate on separate held-out CSV before changing live YAML.')


if __name__ == '__main__':
    main()
