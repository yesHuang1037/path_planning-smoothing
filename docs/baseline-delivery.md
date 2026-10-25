# 本地版本基线交付（2026-10-25）

## 结果及边界

已初始化当前工作区 Git，并将源码、文档、脚本及精选历史证据加入**本地暂存区**。按用户最新要求，没有创建 commit、标签或远端，没有推送云端，也未修改全局 Git 身份。没有提交 SHA，当前仍是可修改的候选基线；不能把暂存等同永久历史或异机备份。

## 文件及原因

| 文件 | 作用 |
| --- | --- |
| `.gitignore` | 排除外部 TDT 仓库、工具目录、构建/安装/日志/缓存及原始 artifacts；原文件仍在本地 |
| `docs/baseline/README.md`、`sources.json` | 固定版本管理边界与三个参考仓库来源，注明原规划包来自有修改的工作树 |
| `docs/baseline/planner-import.json`、`planner-current.json`、`import-to-t02a.patch` | 保存原始 58 文件与当前 62 文件的内容/权限及差异，允许临时目录重建导入版本 |
| `docs/baseline/evidence/`、`evidence.json` | 保存 25 份精选历史证据及哈希，不把历史通过冒充本轮重测 |
| `scripts/verify_baseline.py` | 核验当前包、证据、反向还原、暂存内容及临时导出；遇到偏离报错，不自动更新基线 |
| 本文、`docs/requirements.md` | 记录实际操作、通过/失败和剩余边界 |

没有改算法源码、ROS 配置或原导航工程。临时捕获脚本仅生成 JSON/补丁/证据，不加入项目源码。

## 实际命令与验证

- `git init`：经权限批准成功初始化既有空 `.git`，没有删除或移动该目录；沿用 Git 默认初始分支名称。
- `git add -- .gitignore Exploration.md Phase1Spec.md docs scripts transplant_ws/src/path_searching`：经批准明确暂存，不使用无筛选的 `git add .`；文档更新后再次暂存。
- `python3 /tmp/capture_planner_baseline.py`：核对原 T02 导入哈希及三个来源 tracked diff，再生成基线数据，原始 58 / 当前 62 / 证据 25。
- `python3 scripts/verify_baseline.py`：当前包内容及权限、历史证据、反向补丁还原、暂存与工作树一致性、临时导出均实际检查；最终日志为 `artifacts/T02/baseline-verification-20261025.log`。
- `git diff --cached --stat`、`git status --short`、`git remote -v`、`git tag --list`：检查暂存边界与无远端/无标签；`git rev-parse --verify HEAD` 因尚无提交而失败，符合仅暂存要求，不是仓库损坏。
- `git diff --cached --check`：**未通过**。首次导入的旧源码及存档补丁含尾随空格、空格/Tab 混排；本次不修改源码、不重写证据、不屏蔽检查。新增基线脚本/文档另作限定路径检查。
- 原包完整导入清单、navigation/TDT/SLAM 的 tracked diff 与 T02 前记录一致；原 navigation 七项已有修改保持不变。完整状态列在 `docs/baseline/sources.json`。

本次**未重新构建、未运行算法/ROS/实车测试**；本次通过项仅为版本与内容验证。2026-10-24 的构建、五项 CTest 和独立算法消毒器结果仍按历史证据存档。

## 剩余问题及后续规则

- 没有 commit 或固定标签，不能用提交 SHA 精确引用当前暂存状态；后续 `git add` 会改变候选基线。创建正式提交、标签须另获授权。
- 外部导航/TDT/SLAM 仓库只记录参考身份和差异哈希，并非完整备份；规划包原始导入版本可由补丁重建。
- 未来源码有意变化后，当前清单核验应报偏离；它是固定基线核验，不是每轮都应刷新到“通过”的测试。
- 没有配置远端或异机备份；现有 ESDF 生命周期、安全/失败协议和实机验收问题不在本次修复范围。
