#!/usr/bin/env python3
"""Run repeatable randomized Isaac Sim navigation trials."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description='Batch-test the fire robot on random maps')
    p.add_argument('--runs', type=int, default=10)
    p.add_argument('--start-seed', type=int, default=0)
    p.add_argument('--sensor', choices=['raycast','rgbd'], default='raycast')
    p.add_argument('--max-seconds', type=float, default=300.)
    p.add_argument('--output', type=Path, default=ROOT/'outputs'/'random_maps')
    args = p.parse_args()
    if args.runs < 1:
        p.error('--runs must be at least 1')
    args.output.mkdir(parents=True, exist_ok=True)
    trials = []
    started = time.monotonic()
    for seed in range(args.start_seed, args.start_seed+args.runs):
        trial = args.output/f'seed_{seed}'
        command = [sys.executable, str(ROOT/'run_isaac.py'), '--headless',
                   '--layout', 'random', '--seed', str(seed), '--sensor', args.sensor,
                   '--max-seconds', str(args.max_seconds), '--output', str(trial)]
        print(f'[{len(trials)+1}/{args.runs}] seed {seed}', flush=True)
        completed = subprocess.run(command, check=False)
        result_path = trial/'result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {
            'seed': seed, 'passed': False, 'error': f'runner exited {completed.returncode} without a result'}
        result['exit_code'] = completed.returncode
        trials.append(result)
    passed = sum(bool(t.get('passed')) for t in trials)
    summary = {
        'runs': args.runs, 'passed': passed, 'failed': args.runs-passed,
        'success_rate': passed/args.runs, 'sensor': args.sensor,
        'elapsed_seconds': time.monotonic()-started,
        'failed_seeds': [t.get('seed') for t in trials if not t.get('passed')],
        'trials': trials,
    }
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k != 'trials'},indent=2))
    return 0 if passed == args.runs else 2


if __name__ == '__main__':
    raise SystemExit(main())
