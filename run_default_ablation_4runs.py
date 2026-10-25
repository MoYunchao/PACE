#!/usr/bin/env python3
"""Default map: four runs of each ablation, 0.30 Ah per robot, 30-second pauses."""
from tools.ablation_batch_plan import run_plan


if __name__ == '__main__':
    run_plan('default', ['semantic', 'distance', 'gain', 'risk'],
             '0.30', 'default_ablation_batches')
