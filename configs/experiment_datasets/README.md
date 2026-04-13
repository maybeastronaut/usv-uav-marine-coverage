# experiment_datasets

存放可复用的对比实验数据集目录。

每个子目录尽量自包含：

- 实验说明 `README.md`
- 对比用 `.toml` 配置
- 一份 `batch.toml`
- 运行后生成的 `batch_results.jsonl`、`batch_summary.json`
- 代表性 `html`、逐 seed 的 `events.jsonl / summary.json`

当前保留原则：

- 一个目录只回答一类明确问题
- 除被比较维度外，其余关键模块尽量固定
- 优先使用 3 个可复现实验 seed 做第一轮对比
