import omni
import numpy as np
from omni.isaac.core.world import World
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdLux
# 使用Isaac Sim 4.5的新API
from isaacsim.core.prims import SingleArticulation

class B2Environment:
    """B2机器人仿真环境"""
    
    def __init__(self, num_envs=1):
        self.num_envs = num_envs
        self.world = World(stage_units_in_meters=1.0)
        self.robots = []
        
    def setup_scene(self):
        """设置场景"""
        # 添加地面
        ground_plane = GroundPlane(
            prim_path="/World/groundPlane",
            size=100.0,
            color=np.array([0.5, 0.5, 0.5])
        )
        self.world.scene.add(ground_plane)
        
        # 添加光源 - 使用Isaac Sim 4.5兼容方式
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.DefinePrim("/World/defaultLight", "DistantLight")
        light = UsdLux.DistantLight(light_prim)
        light.CreateIntensityAttr(3000.0)
        light.CreateColorAttr((1.0, 1.0, 1.0))
        
        # 添加B2机器人
        for i in range(self.num_envs):
            self.add_b2_robot(i)
    
    def add_b2_robot(self, env_idx):
        """添加B2机器人"""
        robot_prim_path = f"/World/envs/env_{env_idx}/B2"
        
        # 创建环境容器
        env_prim_path = f"/World/envs/env_{env_idx}"
        create_prim(env_prim_path, "Xform")
        
        # 添加B2机器人USD引用
        b2_usd_path = "/home/xiaohuangfeng/sim/isaac-b2-ros2/model/B2/b2.usd"
        add_reference_to_stage(usd_path=b2_usd_path, prim_path=robot_prim_path)
        
        # 设置机器人位置
        position = np.array([0.0, env_idx * 3.0, 0.8])  # 调整高度避免穿地
        
        # 使用Isaac Sim 4.5的新API创建Articulation
        robot = SingleArticulation(
            prim_path=robot_prim_path,
            name=f"b2_robot_{env_idx}",
            position=position
        )
        
        # 添加到场景
        self.world.scene.add(robot)
        self.robots.append(robot)
        
        # 简单配置，让Isaac Sim自动处理Articulation
        print(f"B2机器人加载完成: {robot_prim_path}")
        
        return robot
    
    
    def reset(self):
        """重置环境"""
        self.world.reset()
        
    def step(self):
        """步进仿真"""
        self.world.step(render=True)
        
    def get_robot_states(self):
        """获取机器人状态"""
        states = []
        for robot in self.robots:
            if robot is not None:
                position, orientation = robot.get_world_pose()
                states.append({
                    'position': position,
                    'orientation': orientation
                })
        return states

def camera_follow(env):
    """相机跟随功能（简化版）"""
    # 暂时不实现相机跟随
    pass