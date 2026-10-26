CSV results are reproducible example outputs, not formal validation or real-vessel safety evidence.

evaluation_v2.csv: historical v2 smoke evaluation, retained for comparison.
evaluation_v3.csv: v3 rule-policy evaluation of 3 episodes with the safety filter enabled; small sample only.
robustness_stress_test.csv: v3 one-seed-per-case smoke matrix covering nominal operation, sensor noise, wind/wave/current, restricted visibility, static obstacle, GNSS loss, compass bias, propulsion loss, rudder jam, and sensor dropout. The limited sample produced 4 safe completions and 6 episodes that did not establish safe completion; 0 vessel collisions and 0 SDA violations. This is deliberately reported as a limitation, not a general success rate.

Run python stress_test.py --episodes-per-case 30 --max-steps 120 for a larger randomized regression sample. Results depend on seeds, settings, heuristic action selection, and the simplified model.
