#!/usr/bin/env python3
"""maze40 risk ablation: four runs, 0.40 Ah per robot, 30-second pauses."""
from tools.ablation_batch_plan import run_plan


if __name__ == '__main__':
    run_plan('maze40', ['risk'], '0.40', 'risk_maze40_batches')
