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
|   `-- discussion_notes.md
|-- pyproject.toml
|-- src/
|   `-- usv_uav_2_0/
|       |-- __init__.py
|       |-- __main__.py
|       |-- agent_overlay.py
|       |-- environment.py
|       `-- viewer.py
`-- tests/
    |-- test_environment.py
    |-- test_smoke.py
    `-- test_viewer.py
```

## File Roles

- `AGENTS.md`: Codex 在本仓库中的开发约束、编码规范、文档同步规则和协作方式说明。
- `README.md`: 项目总览、环境配置方式、运行方式、测试方式和工程结构说明。
- `docx/current_system_flow.md`: 基于当前已有代码整理的系统实际流程说明文档。
- `docx/discussion_notes.md`: 讨论阶段确认的建模方案、状态标记和后续实现依据。
- `pyproject.toml`: Python 项目配置、依赖声明、`pytest` 和 `ruff` 的工具配置入口。
- `src/usv_uav_2_0/__init__.py`: 项目包入口与基础版本信息。
- `src/usv_uav_2_0/__main__.py`: 命令行运行入口，用于生成或打开当前海图 HTML。
- `src/usv_uav_2_0/agent_overlay.py`: 海图上的静态智能体外观示意数据与几何辅助函数。
- `src/usv_uav_2_0/environment.py`: 海域环境、三区带结构、障碍布局、基础监测点、任务热点与伪随机环境生成逻辑。
- `src/usv_uav_2_0/viewer.py`: 海图 HTML/SVG 渲染逻辑，负责底图、障碍、基础监测点、任务热点与静态智能体外观输出。
- `tests/test_environment.py`: 验证默认海域尺寸、三区带范围、障碍环境生成约束以及监测点/热点生成结果。
- `tests/test_smoke.py`: 验证项目包是否能被基础导入。
- `tests/test_viewer.py`: 验证海图 HTML 是否包含关键坐标信息、障碍环境标识、监测点/热点标识与静态智能体标识。

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
The current HTML preview also includes:

- a pseudo-random obstacle layout
- a middle-risk-zone layout that guarantees at least `2` irregular traversable routes without drawing fixed lanes
- `2` nearshore baseline monitoring points
- `5` offshore task hotspots that avoid offshore obstacles and risk areas
- `3` offshore islets and `2` offshore risk areas
- a static visual mockup of `3` USVs and `2` UAVs for appearance inspection only

If you only want to generate the map page without opening a browser:

```bash
python -m usv_uav_2_0 --no-open --output /tmp/usv_uav_sea_map.html
```

If you want reproducible obstacle generation:

```bash
python -m usv_uav_2_0 --seed 20260314
```

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
