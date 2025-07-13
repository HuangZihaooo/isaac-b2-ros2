import torch

# 全局速度命令输入
base_vel_cmd_input = None

def init_base_vel_cmd(num_envs):
    """初始化基座速度命令张量"""
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)

def base_vel_cmd():
    """获取基座速度命令"""
    global base_vel_cmd_input
    if base_vel_cmd_input is not None:
        return base_vel_cmd_input.clone()
    return torch.zeros((1, 3), dtype=torch.float32)

def sub_keyboard_event(event):
    """键盘事件处理器（简化版本）"""
    # 暂时不处理键盘事件，只返回True
    return True