# USV_UAV_2.0

Python project scaffold for follow-up development with Codex.

## Python Version

- Recommended: Python 3.11+
- Current local environment is configured with Python 3.11.15

## Project Structure

```text
.
|-- AGENTS.md
|-- README.md
|-- docx/
|   |-- current_system_flow.md
|   |-- discussion_notes.md
|   `-- generated/
|       `-- sea_map.html
|-- pyproject.toml
|-- src/
|   `-- usv_uav_2_0/
|       |-- __init__.py
|       |-- __main__.py
|       |-- agent_model.py
|       |-- agent_overlay.py
|       |-- environment.py
|       |-- grid.py
|       `-- viewer.py
`-- tests/
    |-- test_agent_model.py
    |-- test_environment.py
    |-- test_grid.py
    |-- test_smoke.py
    `-- test_viewer.py
```

## File Roles

- `AGENTS.md`: Codex 在本仓库中的开发约束、编码规范、文档同步规则和协作方式说明。
- `README.md`: 项目总览、环境配置方式、运行方式、测试方式和工程结构说明。
- `docx/current_system_flow.md`: 基于当前已有代码整理的系统实际流程说明文档。
- `docx/discussion_notes.md`: 讨论阶段确认的建模方案、状态标记和后续实现依据。
- `docx/generated/sea_map.html`: 当前默认生成的 HTML 海图产物，可直接用于查看 `clean` 模式结果。
- `pyproject.toml`: Python 项目配置、依赖声明、`pytest` 和 `ruff` 的工具配置入口。
- `src/usv_uav_2_0/__init__.py`: 项目包入口与基础版本信息。
- `src/usv_uav_2_0/__main__.py`: 命令行运行入口，用于生成或打开当前海图 HTML，并设置页面初始显示模式。
- `src/usv_uav_2_0/agent_model.py`: `USV/UAV` 的最小可用数学模型，包含基础任务状态、简化运动学更新和探测/覆盖半径判定。
- `src/usv_uav_2_0/agent_overlay.py`: 海图上的静态智能体外观示意数据与几何辅助函数。
- `src/usv_uav_2_0/environment.py`: 海域环境、三区带结构、障碍布局、基础监测点、任务热点与伪随机环境生成逻辑。
- `src/usv_uav_2_0/grid.py`: 将连续海域环境映射为 `25m` 规则矩形栅格网络，并提供基于智能体 `footprint` 的动态覆盖状态更新。
- `src/usv_uav_2_0/viewer.py`: 海图 HTML/SVG 渲染逻辑，负责同一 HTML 内 `clean/debug` 切换视图、标签开关、预留轨迹层下的底图、障碍、监测点、热点、静态覆盖预览与静态智能体外观输出。
- `tests/test_agent_model.py`: 验证智能体基础任务状态、简化运动更新以及探测/覆盖半径判定逻辑。
- `tests/test_environment.py`: 验证默认海域尺寸、三区带范围、障碍环境生成约束以及监测点/热点生成结果。
- `tests/test_grid.py`: 验证离散栅格网络的分辨率、坐标映射、环境属性落格结果以及 `footprint` 覆盖映射逻辑。
- `tests/test_smoke.py`: 验证项目包是否能被基础导入。
- `tests/test_viewer.py`: 验证 `clean/debug` 模式下海图 HTML 的关键图层、静态覆盖预览开关与智能体标识是否正确。

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e ".[dev]"
```

## Run Sea Map

```bash
python -m usv_uav_2_0
```

This phase only generates and displays the left-to-right three-zone sea area map.
The generated HTML now includes:

- a built-in `clean/debug` mode switch
- an agent label visibility switch
- a reserved trajectory layer for follow-up dynamic simulation

The default `clean` initial view includes:

- a pseudo-random obstacle layout
- a middle-risk-zone layout that guarantees at least `2` irregular traversable routes without drawing fixed lanes
- `2` nearshore baseline monitoring points
- `5` offshore task hotspots that avoid offshore obstacles and risk areas
- `3` offshore islets and `2` offshore risk areas
- a static visual mockup of `3` USVs and `2` UAVs for appearance inspection only

The environment layer also supports a `25m` rectangular grid network (`40 x 40`), center-point coverage mapping, and confirmed circular footprints (`USV=50m`, `UAV=100m`).

The `debug` view in the same HTML additionally exposes the static coverage preview and footprint outlines, which are only used to verify that `footprint -> grid coverage` alignment is correct.

If you only want to generate the map page without opening a browser:

```bash
python -m usv_uav_2_0 --no-open --output /tmp/usv_uav_sea_map.html
```

If you want reproducible obstacle generation:

```bash
python -m usv_uav_2_0 --seed 20260314
```

If you want the page to open with the debug view selected:

```bash
python -m usv_uav_2_0 --mode debug
```

If you want to generate a leaner HTML without the static validation layers at all, call `build_map_html(..., show_coverage_preview=False, show_footprints=False)` from code.

## Run Tests

```bash
PYTHONPATH=src python -m pytest
```

If `pytest` is not installed in the current environment yet, install project dev dependencies first:

```bash
pip install -e ".[dev]"
```

## Run Lint

```bash
ruff check .
ruff format --check .
```
