# OrbweaversProsCodeLatest

VEX PROS 项目，含底盘 PID（直线/转向）与积分项。

## 构建 (Building)

- **推荐**：在 Cursor/VS Code 中安装 [PROS](https://pros.cs.purdue.edu/) 扩展，用扩展的「Build」或终端里 `pros make` 编译（会使用 PROS 自带工具链，避免缺 `libstdc++`）。
- **命令行**：
  - **macOS**：若直接 `make` 报错 `cannot find -lstdc++`，请先安装完整工具链并把 PROS 工具链置于 PATH 前，或使用 `pros make`。
  - **Windows**：在 [MSYS2](https://www.msys2.org/) 中安装 `arm-none-eabi-gcc`，在 MSYS2 终端里执行 `make`。
  - **Linux**：安装 `arm-none-eabi-gcc` 后执行 `make`。
