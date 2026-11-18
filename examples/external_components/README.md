# 外部组件 wheel 示例

此目录模拟一个独立插件项目，可复制到 MORLA 仓库外构建。包含一台机械臂、米制夹爪、具名字段传感器和双路 JPEG 相机。所有后端都是 fake。

```sh
python -m pip wheel --no-deps . -w dist
python -m pip install --no-deps dist/morla_example_components-1.0.0-py3-none-any.whl
```

在已经安装 MORLA 组合式应用且使用同一 Python 环境时，重启应用后在 Web 的设备预设选择 **External components**。也可以把包中的 `assembly.json` 交给 `morla-system` 编译。夹爪可通过 `morla.scalar_map` 接标量输入，模型文件和 fake IK 工厂展示独立资源和运动学的安装方式。

- `morla_plugin.json` 是随 wheel 安装的静态索引。
- `descriptors.json` 声明四种设备、默认端口和数采字段。
- `pyproject.toml` 注册工厂并将描述、模板和模型打入 wheel。
- `__init__.py` 提供可执行的 fake 工厂。替换为设备协议时实现对应 Adapter，并另外声明 `real` 后端。

参考 [MORLA 组件 SDK](../../docs/architecture_refactor/SDK.md)。
