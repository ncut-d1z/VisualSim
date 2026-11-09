# VisualSim 编译与运行环境命令取证记录

> 记录来源：用户在连续两轮对话中贴出的真实终端命令及输出。记录建立日：2026-11-08；命令实际执行日期未单独提供。**ChatGPT 未在用户的 VirtualBox 上运行这些命令。** 此文档不是编译、CTest、安装或性能验收报告。
>
> 项目：ncut-d1z/VisualSim，Ubuntu 24.04 Linux-only；对照项目 ros2-vio-node/ftr-eskf 仅可读取。实施计划参考 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。

## 一、归档规则

1. 本文件是全部用户实机编译环境**原始命令与原始输出的唯一归档位置**。按 E001、E002 等编号，每条保留逐字可复制的命令、stdout/stderr 原始内容（包括失败、重复行）、记录来源、状态及谨慎的结论。若没有实际退出码，不自行推断。
2. 后续助手请用户执行新命令时，应先在本文件依次新增下一个 E 编号，标记“待执行”，附命令与目的；用户贴回结果时在相同编号下增加原样输出，标记“已反馈、已核查”或“仍待核查”。不覆盖旧输出；需要勘误则追加解释。
3. 不得将用户终端输出误称为助手运行的结果，不能把 CMake Config 文件**存在**解释为实际 find_package 已选中它，不能把某个标准库版本存在解释为已安装的 libvisualsim.so 实际链接了它。
4. 除另有明确授权，环境取证建议应优先使用无 sudo、无安装/编译、无文件修改的只读命令。所有后续 P0/P1 真实编译和运行验收另行登记，不通过本文件提前声称成功。
5. 本文件保存用户提交的**确切输出**，不对重复行、错误行和路径做静默清理。

## 二、用户已确认环境摘要

| 检查 | 用户命令报告的值 | 证据/边界 |
| --- | --- | --- |
| 发行版 | Ubuntu 24.04.5 LTS (Noble Numbat) | E001 |
| 内核 | Linux 7.0.0-38-generic，PREEMPT_DYNAMIC | E002 |
| 架构 | x86_64，GCC target x86_64-pc-linux-gnu | E003、E007 |
| cmake | 4.3.4，/usr/local/cmake-4.3.4/bin/cmake | E004、E005 |
| C++ | GCC 16.2.0，/usr/bin/c++ | E004、E006 |
| glibc | 2.39，Ubuntu 包标识 2.39-0ubuntu8.9 | E009 |
| Eigen，pkg-config | 3.4.0 | E010；另有 Eigen 5.0.1 build 目录，**未确认被选中** |
| OpenCV，pkg-config opencv4 | 4.14.0 | E010；opencv4/opencv5 的 CMake Config 均存在，**尚未确认 CMake 选中哪一个** |
| yaml-cpp，pkg-config | 0.9.0 | E010 |
| Sophus | 找到 /usr/local/share/sophus/cmake/SophusConfig.cmake | E013；**尚未确认版本** |
| GCC 标准库 realpath | /usr/local/gcc-16/lib64/libstdc++.so.6.0.36 | E014 |
| libstdc++ 动态库缓存 | /usr/local/gcc-16/lib64 和 /lib/x86_64-linux-gnu 均有 libstdc++.so.6 | E015；不代表具体进程实际选择 |
| GLIBCXX 符号 | 用户输出含 GLIBCXX_3.4.36 | E016；其他 GLIBCXX_DEBUG_MESSAGE_LENGTH 等不是版本号 |

## 三、上一轮对话中的原始执行记录

所有原文均由用户以 ros@VirtualBox:~/Dev/FastMono2$ 提示符贴出；未提供实际 shell 退出码。

### E001 — Ubuntu 系统信息（已核查）

命令：

~~~bash
cat /etc/os-release | grep -E '^(NAME|VERSION|VERSION_ID)='
~~~

输出：

~~~text
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04.5 LTS (Noble Numbat)"
~~~

### E002 — 内核信息（已核查）

~~~bash
uname -a
~~~

输出：

~~~text
Linux VirtualBox 7.0.0-38-generic #38~24.04.4-Ubuntu SMP PREEMPT_DYNAMIC Mon Sep 14 16:37:11 UTC 2 x86_64 x86_64 x86_64 GNU/Linux
~~~

### E003 — 主机架构（已核查）

~~~bash
uname -m
~~~

输出：

~~~text
x86_64
~~~

### E004 — 命令搜索路径（已核查）

~~~bash
command -v cmake c++
~~~

输出：

~~~text
/usr/local/cmake-4.3.4/bin/cmake
/usr/bin/c++
~~~

### E005 — CMake 版本（已核查）

~~~bash
cmake --version
~~~

输出：

~~~text
cmake version 4.3.4

CMake suite maintained and supported by Kitware (kitware.com/cmake).
~~~

### E006 — 编译器版本（已核查）

~~~bash
c++ --version
~~~

输出：

~~~text
c++ (GCC) 16.2.0
Copyright (C) 2026 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.
There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
~~~

### E007 — GCC 编译目标（已核查）

~~~bash
c++ -dumpmachine
~~~

输出：

~~~text
x86_64-pc-linux-gnu
~~~

### E008 — GCC 查找标准库路径（已核查）

~~~bash
c++ -print-file-name=libstdc++.so.6
~~~

输出：

~~~text
/usr/local/gcc-16/lib/gcc/x86_64-pc-linux-gnu/16.2.0/../../../../lib64/libstdc++.so.6
~~~

### E009 — glibc 版本（已核查）

~~~bash
ldd --version | head -n 1
~~~

输出：

~~~text
ldd (Ubuntu GLIBC 2.39-0ubuntu8.9) 2.39
~~~

### E010 — 三个 pkg-config 依赖版本（已核查）

~~~bash
if command -v pkg-config >/dev/null 2>&1; then
  for pkg in eigen3 opencv4 yaml-cpp; do
    printf '%s: ' "$pkg"
    pkg-config --modversion "$pkg" 2>&1 || true
  done
fi
~~~

输出：

~~~text
eigen3: 3.4.0
opencv4: 4.14.0
yaml-cpp: 0.9.0
~~~

### E011 — 最初的 Config 搜索命令，Bash 报错（失败证据保留）

用户粘贴的命令：

~~~bash
find /usr /usr/local /opt -type f   ( -name 'SophusConfig.cmake' -o -name 'sophus-config.cmake'      -o -name 'Eigen3Config.cmake' -o -name 'OpenCVConfig.cmake'      -o -name 'yaml-cpp-config.cmake' ) -print 2>/dev/null | head -n 80
~~~

输出：

~~~text
bash: syntax error near unexpected token '('
~~~

分析：这里未对 Bash 特殊符号圆括号转义，命令未成功运行。这**不证明** Sophus 或其他依赖不存在；后续 E012/E013 给出修正和结果。

## 四、本轮对话中的原始执行记录

### E012 — 修正后的 CMake Config 文件搜索（成功返回输出）

~~~bash
find /usr /usr/local /opt -type f \
  \( -name 'SophusConfig.cmake' \
     -o -name 'sophus-config.cmake' \
     -o -name 'Eigen3Config.cmake' \
     -o -name 'OpenCVConfig.cmake' \
     -o -name 'yaml-cpp-config.cmake' \) \
  -print 2>/dev/null | head -n 80
~~~

### E013 — E012 的完整原始输出（已核查）

~~~text
/usr/local/share/sophus/cmake/SophusConfig.cmake
/usr/local/share/eigen3/cmake/Eigen3Config.cmake
/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake
/usr/local/lib/cmake/opencv5/OpenCVConfig.cmake
/usr/local/lib/cmake/yaml-cpp/yaml-cpp-config.cmake
/usr/local/share/sophus/cmake/SophusConfig.cmake
/usr/local/share/eigen3/cmake/Eigen3Config.cmake
/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake
/usr/local/lib/cmake/opencv5/OpenCVConfig.cmake
/usr/local/lib/cmake/yaml-cpp/yaml-cpp-config.cmake
/opt/eigen-5.0.1/build/Eigen3Config.cmake
~~~

注意：重复路径**按用户原样保留**。find 同时搜索 /usr 和其子目录 /usr/local，搜索范围重叠可能造成重复，但本文件不把相同路径解释为两个不同安装实例。opencv5 Config 和 Eigen5 构建树的存在**不代表**已经用于 VisualSim 构建。Sophus Config 存在，但还需要版本及依赖目标进一步证据。

### E014 — GCC 标准库符号链接真实目标（已核查）

~~~bash
readlink -f "$(c++ -print-file-name=libstdc++.so.6)"
~~~

输出：

~~~text
/usr/local/gcc-16/lib64/libstdc++.so.6.0.36
~~~

### E015 — 动态链接缓存中的 libstdc++ 项（已核查）

~~~bash
ldconfig -p | grep libstdc++
~~~

输出：

~~~text
	libstdc++.so.6 (libc6,x86-64) => /usr/local/gcc-16/lib64/libstdc++.so.6
	libstdc++.so.6 (libc6,x86-64) => /lib/x86_64-linux-gnu/libstdc++.so.6
	libstdc++.so.6 (libc6) => /lib32/libstdc++.so.6
	libstdc++.so (libc6,x86-64) => /usr/local/gcc-16/lib64/libstdc++.so
~~~

尚不能据此判断未来安装的 libvisualsim.so 或独立消费者实际加载哪一个库；P1/P6 构建后需要对真实 ELF/进程检验。

### E016 — 该 C++ 标准库中的 GLIBCXX 字符串（已核查）

~~~bash
strings "$(c++ -print-file-name=libstdc++.so.6)" \
  | grep '^GLIBCXX_' | sort -V | tail -n 5
~~~

输出：

~~~text
GLIBCXX_3.4.34
GLIBCXX_3.4.35
GLIBCXX_3.4.36
GLIBCXX_DEBUG_MESSAGE_LENGTH
GLIBCXX_TUNABLES
~~~

注意：版本字符串最大可见为 GLIBCXX_3.4.36；最后两行属于非版本字符串，不能算成更高版本。

## 五、环境兼容性判断与待确认事项

- **确认存在**：Ubuntu 24.04.5、GCC 16.2.0、CMake 4.3.4、Eigen pkg-config 3.4.0、OpenCV pkg-config 4.14.0、yaml-cpp pkg-config 0.9.0、Sophus 的 CMake Config 文件。
- **未确认**：Sophus 具体版本、Sophus::Sophus 是否真正能通过目标构建配置解析；OpenCV4/OpenCV5、Eigen3/Eigen5 在实际 CMake 构建时会选择哪个 Config；库和消费者的 C++ ABI 与动态链接器最终路径。
- **不能据此宣称**：P0–P6 已实施；生产代码已编译；CTest、独立消费者、安装或数值测试已通过。
- **不需要降级任何已知版本**：设计继续只面向 Ubuntu 24.04 / Linux，不允许 Windows，也不依赖 ROS。

## 六、后续命令专用追加区

**下一个可分配编号：E017。** 此处目前没有新的待执行命令。以后我建议你运行任何新的本机检查时，应在本文件先追加 E017、E018……，包含目的、完整可复制命令、状态“待执行”；收到终端结果后按编号追加完整 stdout/stderr、退出状态（如提供）与审查结论，**不改写 E001–E016 的原始记录**。

新条目采用如下字段：编号与标题；命令；建议执行原因；输出（未执行时写“待提供”）；退出码（未提供时不可推断）；状态；具体结论；相关的 PLAN 阶段。新任务如有修改系统、安装软件或构建副作用，应事先如实说明，而不混入纯只读检查。

本文件在模型可访问的 GitHub 目标仓库中持续维护；原始用户输出仅来自聊天提供的文本。任何后续结果都需明确区分用户提供的观察与模型依据观察所得的推断。
