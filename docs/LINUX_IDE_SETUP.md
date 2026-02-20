# Linux 下 IDE 报错说明与处理

## 为什么在 Linux 上会报很多错？

你在 Linux 上看到的这些错误（如「命名空间 'pros' 没有成员」「std 没有成员 'vector'」「未定义标识符 IMU_CAL_TIMEOUT_MS」等）**绝大多数是 IDE 的 IntelliSense 配置问题**，不是源码写错。

原因主要有：

1. **只有 macOS 的 C++ 配置**  
   项目里的 `.vscode/c_cpp_properties.json` 之前只配置了 Mac：编译器路径是 Mac 的、带 `macFrameworkPath`。在 Linux 上 Cursor/VSCode 要么用不到这套配置，要么用了错误路径，导致：
   - 找不到 **PROS 头文件**（`pros::Controller`、`Motor`、`Imu` 等）
   - 找不到 **交叉编译器的 C++ 标准库**（`std::vector` 等）
   - 找不到项目里的 **`include/`**（例如 `config.hpp` 里的 `IMU_CAL_TIMEOUT_MS`）

2. **Linux 和 Mac 路径不同**  
   - 编译器：Mac 上可能是 PROS 自带的 `pros-toolchain-macos/.../arm-none-eabi-g++`，Linux 上一般是系统或 PROS 安装的 `arm-none-eabi-g++`。
   - 没有为 Linux 单独配一套「编译器路径 + 包含路径」时，IntelliSense 就解析不了 `pros::*` 和 `std::*`，从而报出一大串「没有成员」「未定义标识符」等。

3. **和实际编译是两回事**  
   在终端里执行 `pros make`（或 `make`）时，用的是 Makefile 和系统 PATH 里的 `arm-none-eabi-g++`，所以**在 Linux 上实际编译往往能通过**，只是编辑器里的红线和错误列表会很多。

---

## 已做的修改

- 在 **`.vscode/c_cpp_properties.json`** 里增加了一个 **Linux** 配置：
  - `compilerPath`: 先设为 **`/usr/bin/arm-none-eabi-g++`**（常见安装位置）。
  - `includePath`: 使用 **`${workspaceFolder}/**`** 和 **`${workspaceFolder}/include`**，这样在 Linux 上也能找到 `include/config.hpp` 和 PROS 头文件。
  - 去掉了 Mac 专用的 `macFrameworkPath`，并使用了适合 Linux 的 `intelliSenseMode`。

这样在 Linux 上打开项目时，只要选中「Linux」这一套配置，IntelliSense 就会用正确的编译器与包含路径去解析，大部分报错会消失。

---

## 你需要做的（在 Linux 上）

1. **确认已安装 arm-none-eabi 工具链**

   在终端执行：

   ```bash
   which arm-none-eabi-g++
   ```

   - 若输出路径（例如 `/usr/bin/arm-none-eabi-g++`），说明已安装。若项目或 PROS 把工具链装到别处（例如 `~/.local/share/pros-cli/...`），记下该路径。
   - 若没有输出，需要先安装，例如：
     - Ubuntu/Debian: `sudo apt install gcc-arm-none-eabi g++-arm-none-eabi`
     - Fedora: `sudo dnf install arm-none-eabi-gcc-cs arm-none-eabi-gcc-cs-c++`
     - 或按 [PROS 文档](https://pros.cs.purdue.edu/v5/getting-started/index.html) 用 PROS CLI 安装工具链。

2. **（如需要）改编译器路径**

   若你的 `arm-none-eabi-g++` 不在 `/usr/bin/`，请打开 **`.vscode/c_cpp_properties.json`**，在 **Linux** 那一段里把 `compilerPath` 改成你本机的路径，例如：

   ```json
   "compilerPath": "/你的路径/arm-none-eabi-g++"
   ```

3. **让 Cursor/VSCode 使用 Linux 配置**

   - 按 `Ctrl+Shift+P`（或 `Cmd+Shift+P`）打开命令面板。
   - 输入并选择：**C/C++: Edit Configurations (UI)** 或 **C/C++: Select a Configuration...**。
   - 在「Configuration」里选 **Linux**（不要选 macos-gcc-arm64）。
   - 若提示「选择 IntelliSense 配置」，同样选 **Linux**。

4. **重新解析一次**

   - 保存 `c_cpp_properties.json` 后，可关闭再重新打开 `main.cpp`，或执行 **C/C++: Reset IntelliSense Database**，让 IDE 用新配置重新扫一遍。

---

## 小结

| 现象 | 原因 | 处理 |
|------|------|------|
| `pros` 没有成员、`std::vector` 报错等 | Linux 下未配置编译器/包含路径 | 使用新增的「Linux」配置并确认 `compilerPath` |
| `IMU_CAL_TIMEOUT_MS` 未定义 | 未找到 `include/config.hpp` | `includePath` 已包含 `${workspaceFolder}/include`，选 Linux 配置即可 |
| 终端 `pros make` 能过、IDE 仍报错 | 编译用 Makefile，IDE 用 c_cpp_properties | 按上面步骤选对 Linux 配置并重置 IntelliSense |

按上述步骤在 Linux 上配置好后，这些报错会明显减少或消失；若仍有少量红线，多半是 IntelliSense 的延迟或缓存，不影响实际 `pros make` 编译。
