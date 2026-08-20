# 持续集成质量门禁

项目提供一条与平台无关的质量门禁命令。它只依赖项目本来就需要的
CMake 3.16+、Git 和 C99 编译器，既可在 Windows/MinGW 本机运行，也可放入
Linux CI 构建步骤。

```powershell
cmake -DQUALITY_BUILD_DIR=build -DQUALITY_CONFIG=Debug -P cmake/quality_gate.cmake
```

门禁依次执行：

1. `git diff --check`，拒绝空白错误；
2. 检查 Git 已跟踪文件，拒绝 CMake 缓存、可执行文件、目标文件和日志等构建产物；
3. 检查源码及构建配置，拒绝朋友电脑遗留的 Windows 绝对路径；
4. 在仓库唯一约定的 `build/` 目录中重新配置并干净构建；
5. 为全部目标启用编译器警告，并在门禁中将警告视为错误；
6. 运行完整 CTest 回归集，任意失败都会返回非零退出码。

只运行快速仓库检查时使用：

```powershell
cmake -DQUALITY_CHECK_ONLY=ON -P cmake/quality_gate.cmake
```

`QUALITY_BUILD_DIR` 必须是仓库内的 `build/` 或它的子目录。脚本不会删除目录，
但会用 `cmake --build --clean-first` 清理当前 CMake 目标，避免旧目标文件掩盖问题。
已有 `CMakeCache.txt` 时沿用原生成器；全新 Windows 构建目录在找到
`mingw32-make` 时自动使用 `MinGW Makefiles`。

## 接入 Gitee 流水线

Gitee 的流水线功能需要先在仓库网页中开通并选择运行环境。开通后，在流水线的
自定义命令步骤中直接运行上面的完整门禁命令，并把触发条件设为功能分支推送及
合并请求。Linux 镜像应预装 Git、CMake 3.16+ 和 C 编译器。

仓库暂不手写 Gitee 专用 YAML：在未开通流水线、未由 Gitee 确认当前配置格式和
运行镜像前提交平台文件，无法在本地验证且可能根本不会执行。仓库内门禁保持单一
事实来源；以后网页生成流水线配置时，只调用这条命令，不复制构建与测试逻辑。

建议将该步骤设为合并请求的必需检查。这样 `main` 只接收能够无警告编译并通过
全部回归测试的分支。
