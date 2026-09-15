# 文档维护

用户文档源文件位于 `docs/sdk/`，入口为 [SDK 文档首页](sdk/index.md)。
MkDocs 构建只包含该目录；研发状态、历史实验与实现分析不进入用户导航或搜索。

## 浏览与构建

文档可独立构建，不需要安装 SDK 原生库或连接设备。
在仓库根目录创建专用环境：

```bash
python -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt
.venv-docs/bin/python -m mkdocs serve --dev-addr 127.0.0.1:8000
```

打开 `http://127.0.0.1:8000`，可使用章节导航、页内目录、搜索和代码复制。
`jieba` 在构建时进行中文分词；站内搜索也支持 `set_position` 等完整 API 名称。

生成静态文档：

```bash
.venv-docs/bin/python -m mkdocs build --strict
```

产物为 `site/`。可以随交付物一起提供，通过任意静态 HTTP 服务浏览。
本命令仅构建，不部署或公开发布文档。
构建方式依据 [MkDocs 配置](https://www.mkdocs.org/user-guide/configuration/) 与
[Material 导航](https://squidfunk.github.io/mkdocs-material/setup/setting-up-navigation/)。
搜索配置见 [Material 搜索插件](https://squidfunk.github.io/mkdocs-material/plugins/search/)。

## 内容约定

- 快速开始：完整的最小使用路径，不解释内部实现和项目历史。
- 使用指南：按任务组织；说明前提、代码、结果和必要的行为约束。
- API 参考：每个方法独立描述签名、参数、返回值、异常和示例；字段单位明确。
- 设备维护：使用交付文件与设备选择器，不依赖开发者机器上的绝对路径或分支。
- 开发记录：构建证据、测试结果、根因假设、待办和设计决策位于 `docs/development/`。

用户页不记录个人对话、开发进度或未来功能承诺。
发布版本时同步维护根目录的完整 `CHANGELOG.md` 与 `docs/sdk/changelog.md` 的客户变更摘要；
尚未发布的修改放入“未发布”，不提前填写发布日期或声称交付包已包含。
已有 `docs/USAGE.md` 等入口保留为跳转页，避免旧链接失效；正文只维护一份。

## 检查

运行 `python scripts/check_sdk_docs.py` 核对示例语法、API 参数覆盖与本地链接，
再运行严格构建。预览中检查导航、表格、代码块和搜索。
这些检查不连接设备，不执行文档中的运动示例。
