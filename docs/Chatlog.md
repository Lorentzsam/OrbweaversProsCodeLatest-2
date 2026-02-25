## 2026-02-20 19:56 安装 Python 3.13

- **对话标题**：使用 Homebrew 安装 Python 3.13
- **用户需求**：帮忙安装 Python 3.13
- **解决方案**：通过 Homebrew 执行 `brew install python@3.13`，将已有 3.13.2 升级为 3.13.12；验证 `python3.13 --version` 可用。
- **代码改动**：无（仅系统安装，未修改项目文件）
- **状态标签**：✅完成

## 2026-02-20 替换 opcontrol 操控逻辑

- **对话标题**：使用指定文件的 controller 操控逻辑
- **用户需求**：用提供的 main.cpp 中的 controller 操控逻辑替换当前 opcontrol，不更改其他部分
- **解决方案**：将 opcontrol() 替换为提供的版本——手柄等待 while(!connected)、右轮 0.8 系数、Intake 127/-127、Wing 改为 RIGHT/LEFT、移除 updateOdometry/drawVelocityGraph/LCD 里程计
- **代码改动**：`src/main.cpp` — 修改 opcontrol() 函数
- **状态标签**：✅完成

---
