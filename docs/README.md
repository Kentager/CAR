# 智能车项目文档

## 文档说明

本目录包含智能车项目的详细技术文档和使用手册。

## 文档列表

### 1. [control_simple_manual.md](control_simple_manual.md) ⭐ 推荐
**简洁控制模块使用手册 (v2.0)**

- **更新日期**: 2026-04-06
- **内容**:
  - 三步快速上手
  - 完整API说明
  - 实用使用示例
  - 典型应用流程
  - 参数调优指南
  - 常见问题解答

**特点**: API简洁，使用简单，适合快速开发

### 2. [control_module_manual.md](control_module_manual.md)
**智能车双环PID控制模块使用手册 (v1.1)**

- **更新日期**: 2026-04-06
- **内容**:
  - 模块概述和控制架构
  - 详细的接口说明
  - 完整的使用流程
  - 参数配置和调试建议
  - 注意事项和常见问题

**特点**: 功能完整，适合需要深入了解的开发者

### 3. [main_control_test.c](../USER/Src/main_control_test.c)
**控制模块测试案例**

- **版本**: 1.0
- **更新日期**: 2026-04-06
- **内容**:
  - 全面的自动化测试
  - 性能评估和精度测试

### 4. [main_control_simple.c](../USER/Src/main_control_simple.c)
**简洁控制模块示例代码**

- **版本**: 1.0
- **更新日期**: 2026-04-06
- **内容**:
  - 简单易懂的使用示例
  - 常见场景演示
  - 状态监控示例

## 快速开始

### 查看简洁版手册（推荐新手）
```bash
cat docs/control_simple_manual.md
```

### 查看完整版手册
```bash
cat docs/control_module_manual.md
```

### 运行简洁示例
```bash
# 使用 main_control_simple.c
# 三步上手：
# 1. Control_Init();
# 2. Control_SetSpeed(0.5f); Control_Enable();
# 3. Control_Update(); // 10ms周期
```

### 运行测试案例
1. 将 `USER/Src/main_control_test.c` 添加到项目编译
2. 编译并烧录到智能车
3. 通过串口查看测试结果

## 文档更新日志

### 2026-04-06
- **重写control模块**: 简化API，仅使用MPU6050 DMP角度
- **新增**: `control_simple_manual.md` - 简洁控制模块使用手册
- **新增**: `main_control_simple.c` - 简洁使用示例代码
- **更新**: `control_module_manual.md` - 保留作为详细参考
- **更新**: `README.md` - 更新文档索引和快速开始指南

## 推荐使用流程

### 新手入门
1. 阅读 `control_simple_manual.md`
2. 运行 `main_control_simple.c` 示例
3. 根据需要修改参数
4. 开始自己的项目

### 深入开发
1. 阅读 `control_simple_manual.md` 了解基础
2. 阅读 `control_module_manual.md` 了解细节
3. 查看 `main_control_test.c` 学习测试方法
4. 根据需求调整PID参数

## 注意事项

1. **文档版本**: 请确保查看最新版本的文档
2. **代码同步**: 测试案例代码需要与当前项目代码版本匹配
3. **参数调优**: 文档中的参数为默认值，需要根据实际硬件调优
4. **安全第一**: 运行测试案例时请确保在安全环境中进行

## 技术支持

如有问题或建议，请通过以下方式反馈：
- GitHub Issues: 项目仓库
- 项目文档: 查看 CLAUDE.md 获取更多项目信息

---

**文档维护**: Claude Sonnet 4.6
**最后更新**: 2026-04-06
