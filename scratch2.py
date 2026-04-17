import json

with open("outputs/rl/ppo_usv_offshore_hotspot_eval_seed_20260326_summary.json") as f:
    data = json.load(f)
    print(data.get("experiment_config"))
