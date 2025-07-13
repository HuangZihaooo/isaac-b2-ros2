#!/usr/bin/env python3
"""
测试导入模块，验证修复是否成功
"""

print("测试导入模块...")

try:
    import b2.b2_ctrl as b2_ctrl
    print("✅ 成功导入 b2.b2_ctrl")
except Exception as e:
    print(f"❌ 导入 b2.b2_ctrl 失败: {e}")

try:
    from b2.b2_env import B2Environment, camera_follow
    print("✅ 成功导入 b2.b2_env")
except Exception as e:
    print(f"❌ 导入 b2.b2_env 失败: {e}")

try:
    import env.sim_env as sim_env
    print("✅ 成功导入 env.sim_env")
except Exception as e:
    print(f"❌ 导入 env.sim_env 失败: {e}")

try:
    import torch
    print("✅ 成功导入 torch")
except Exception as e:
    print(f"❌ 导入 torch 失败: {e}")

print("\n模块导入测试完成！")
print("\n现在可以尝试运行:")
print("python isaac_b2_ros2.py")