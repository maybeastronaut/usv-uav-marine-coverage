"""Train a centralized RLlib PPO policy for USV high-level task allocation."""

from __future__ import annotations

import argparse
import json
import shutil
import tomllib
from dataclasses import asdict, dataclass
from pathlib import Path

import ray
from ray.rllib.algorithms.ppo import PPOConfig

from usv_uav_marine_coverage.rl import register_centralized_usv_allocator_env


@dataclass(frozen=True)
class PpoTrainingConfig:
    output_dir: Path
    base_config_path: Path | None
    scenario_name: str
    episode_steps: int
    dt_seconds: float
    seed_sequence: tuple[int, ...]
    iterations: int
    checkpoint_every: int
    num_env_runners: int
    rollout_fragment_length: int
    train_batch_size: int
    minibatch_size: int
    num_epochs: int
    lr: float
    gamma: float
    lambda_: float
    entropy_coeff: float


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Training TOML config path.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Optional output directory override. Defaults to the config value.",
    )
    args = parser.parse_args()

    training_config = load_training_config(args.config)
    if args.output_dir is not None:
        training_config = PpoTrainingConfig(
            **{**asdict(training_config), "output_dir": args.output_dir.resolve()}
        )

    training_config.output_dir.mkdir(parents=True, exist_ok=True)
    env_name = register_centralized_usv_allocator_env()
    ray.init(ignore_reinit_error=True, include_dashboard=False)
    try:
        algorithm = (
            PPOConfig()
            .environment(
                env=env_name,
                env_config={
                    "base_config_path": (
                        None
                        if training_config.base_config_path is None
                        else str(training_config.base_config_path)
                    ),
                    "scenario_name": training_config.scenario_name,
                    "episode_steps": training_config.episode_steps,
                    "dt_seconds": training_config.dt_seconds,
                    "seed_sequence": list(training_config.seed_sequence),
                },
            )
            .framework("torch")
            .resources(num_gpus=0)
            .env_runners(
                num_env_runners=training_config.num_env_runners,
                rollout_fragment_length=training_config.rollout_fragment_length,
            )
            .training(
                train_batch_size=training_config.train_batch_size,
                minibatch_size=training_config.minibatch_size,
                num_epochs=training_config.num_epochs,
                lr=training_config.lr,
                gamma=training_config.gamma,
                lambda_=training_config.lambda_,
                entropy_coeff=training_config.entropy_coeff,
            )
            .build()
        )

        metrics: list[dict[str, object]] = []
        latest_checkpoint_path: Path | None = None
        checkpoint_root = training_config.output_dir / "checkpoints"
        checkpoint_root.mkdir(parents=True, exist_ok=True)
        for iteration in range(1, training_config.iterations + 1):
            result = algorithm.train()
            metrics.append(
                {
                    "iteration": iteration,
                    "episode_reward_mean": result.get("env_runners", {}).get(
                        "episode_return_mean",
                        result.get("episode_reward_mean"),
                    ),
                    "episodes_total": result.get("num_episodes_lifetime", result.get("episodes_total")),
                    "timesteps_total": result.get("timesteps_total"),
                }
            )
            print(
                "[RL] iteration="
                f"{iteration} reward_mean={metrics[-1]['episode_reward_mean']} "
                f"timesteps={metrics[-1]['timesteps_total']}"
            )
            if iteration % training_config.checkpoint_every != 0 and iteration != training_config.iterations:
                continue
            latest_checkpoint_path = _resolve_checkpoint_path(
                algorithm.save(str(checkpoint_root))
            )
            _sync_latest_checkpoint(latest_checkpoint_path, training_config.output_dir)

        summary_path = training_config.output_dir / "training_summary.json"
        summary_path.write_text(
            json.dumps(
                {
                    "training": {
                        "output_dir": str(training_config.output_dir),
                        "base_config_path": (
                            None
                            if training_config.base_config_path is None
                            else str(training_config.base_config_path)
                        ),
                        "scenario_name": training_config.scenario_name,
                        "episode_steps": training_config.episode_steps,
                        "dt_seconds": training_config.dt_seconds,
                        "seed_sequence": list(training_config.seed_sequence),
                        "iterations": training_config.iterations,
                        "checkpoint_every": training_config.checkpoint_every,
                    },
                    "latest_checkpoint_path": (
                        None if latest_checkpoint_path is None else str(latest_checkpoint_path)
                    ),
                    "stable_checkpoint_path": str(training_config.output_dir / "latest_checkpoint"),
                    "metrics": metrics,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        if latest_checkpoint_path is not None:
            print(f"[RL] latest checkpoint: {latest_checkpoint_path}")
            print(f"[RL] stable checkpoint: {training_config.output_dir / 'latest_checkpoint'}")
    finally:
        ray.shutdown()


def load_training_config(path: Path) -> PpoTrainingConfig:
    raw = tomllib.loads(path.read_text(encoding="utf-8"))
    training_raw = _expect_table(raw, "training")
    environment_raw = _expect_table(raw, "environment")

    output_dir = _resolve_relative(path, Path(_read_non_empty_str(training_raw, "output_dir")))
    base_config_value = environment_raw.get("base_config_path")
    base_config_path = None
    if base_config_value is not None:
        if not isinstance(base_config_value, str) or not base_config_value:
            raise ValueError("environment.base_config_path must be a non-empty string")
        base_config_path = _resolve_relative(path, Path(base_config_value))
    seed_sequence = environment_raw.get("seed_sequence", list((20260401, 20260402, 20260403)))
    if not isinstance(seed_sequence, list) or not seed_sequence:
        raise ValueError("environment.seed_sequence must be a non-empty list")
    return PpoTrainingConfig(
        output_dir=output_dir,
        base_config_path=base_config_path,
        scenario_name=_read_non_empty_str(environment_raw, "scenario_name"),
        episode_steps=_read_int(environment_raw, "episode_steps", minimum=1),
        dt_seconds=_read_float(environment_raw, "dt_seconds", minimum=0.0),
        seed_sequence=tuple(int(seed) for seed in seed_sequence),
        iterations=_read_int(training_raw, "iterations", minimum=1),
        checkpoint_every=_read_int(training_raw, "checkpoint_every", minimum=1),
        num_env_runners=_read_optional_int(
            training_raw,
            "num_env_runners",
            default=0,
            minimum=0,
        ),
        rollout_fragment_length=_read_int(training_raw, "rollout_fragment_length", minimum=1),
        train_batch_size=_read_int(training_raw, "train_batch_size", minimum=1),
        minibatch_size=_read_int(training_raw, "minibatch_size", minimum=1),
        num_epochs=_read_int(training_raw, "num_epochs", minimum=1),
        lr=_read_float(training_raw, "lr", minimum=0.0),
        gamma=_read_float(training_raw, "gamma", minimum=0.0),
        lambda_=_read_float(training_raw, "lambda", minimum=0.0),
        entropy_coeff=_read_float(training_raw, "entropy_coeff", minimum=0.0),
    )


def _sync_latest_checkpoint(source_checkpoint: Path, output_dir: Path) -> None:
    target_dir = output_dir / "latest_checkpoint"
    if target_dir.exists():
        shutil.rmtree(target_dir)
    shutil.copytree(source_checkpoint, target_dir)


def _resolve_checkpoint_path(save_result: object) -> Path:
    if isinstance(save_result, (str, Path)):
        return Path(save_result).resolve()
    checkpoint = getattr(save_result, "checkpoint", None)
    if checkpoint is None:
        raise TypeError(f"Unsupported checkpoint result type: {type(save_result)!r}")
    checkpoint_path = getattr(checkpoint, "path", None)
    if checkpoint_path is not None:
        return Path(checkpoint_path).resolve()
    to_directory = getattr(checkpoint, "to_directory", None)
    if callable(to_directory):
        return Path(to_directory()).resolve()
    raise TypeError(f"Checkpoint object does not expose a usable local path: {type(checkpoint)!r}")


def _expect_table(raw: dict[str, object], key: str) -> dict[str, object]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Config table [{key}] is required")
    return value


def _read_non_empty_str(raw: dict[str, object], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    return value


def _read_int(raw: dict[str, object], key: str, *, minimum: int) -> int:
    value = raw.get(key)
    if not isinstance(value, int):
        raise ValueError(f"{key} must be an integer")
    if value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return value


def _read_optional_int(
    raw: dict[str, object],
    key: str,
    *,
    default: int,
    minimum: int,
) -> int:
    value = raw.get(key)
    if value is None:
        return default
    if not isinstance(value, int):
        raise ValueError(f"{key} must be an integer when provided")
    if value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return value


def _read_float(raw: dict[str, object], key: str, *, minimum: float) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a float")
    float_value = float(value)
    if float_value < minimum:
        raise ValueError(f"{key} must be >= {minimum}")
    return float_value


def _resolve_relative(base_path: Path, candidate: Path) -> Path:
    if candidate.is_absolute():
        return candidate
    return (base_path.parent / candidate).resolve()


if __name__ == "__main__":
    main()
