from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
import omni.isaac.core.utils.prims as prim_utils
import numpy as np

def add_semantic_label():
    """添加语义标签（简化版）"""
    # 暂时不添加语义标签
    pass

def create_simple_obstacles():
    """创建简单的障碍物"""
    from pxr import UsdPhysics, UsdGeom
    from omni.isaac.core.objects import DynamicCuboid
    
    # 创建几个简单的立方体作为障碍物
    for i in range(5):
        obstacle_name = f"Obstacle_{i}"
        position = np.array([i * 3.0 - 6.0, (i * -1) * 3.0 , 1.0])
        
        # 使用DynamicCuboid创建障碍物，确保有正确的物理属性
        obstacle = DynamicCuboid(
            prim_path=f"/World/{obstacle_name}",
            name=obstacle_name,
            position=position,
            scale=np.array([0.5, 0.5, 2.0]),
            color=np.array([0.7, 0.4, 0.2])  # 棕色
        )
        
        # 在Isaac Sim 4.5中，DynamicCuboid自动具有物理属性，无需手动设置

def create_obstacle_sparse_env():
    """创建稀疏障碍物环境（简化版）"""
    add_semantic_label()
    create_simple_obstacles()

def create_obstacle_medium_env():
    """创建中等障碍物环境（简化版）"""
    from omni.isaac.core.objects import DynamicCuboid
    
    add_semantic_label()
    create_simple_obstacles()
    
    # 添加更多障碍物
    for i in range(5, 10):
        obstacle_name = f"Obstacle_{i}"
        position = np.array([i * 2.0 - 10.0, 3.0, 1.0])
        
        obstacle = DynamicCuboid(
            prim_path=f"/World/{obstacle_name}",
            name=obstacle_name,
            position=position,
            scale=np.array([0.5, 0.5, 2.0]),
            color=np.array([0.7, 0.4, 0.2])
        )

def create_obstacle_dense_env():
    """创建密集障碍物环境（简化版）"""
    from omni.isaac.core.objects import DynamicCuboid
    
    add_semantic_label()
    create_simple_obstacles()
    
    # 创建更密集的障碍物
    for i in range(10):
        for j in range(3):
            obstacle_name = f"Obstacle_{i}_{j}"
            position = np.array([i * 2.0 - 10.0, j * 2.0 - 2.0, 1.0])
            
            obstacle = DynamicCuboid(
                prim_path=f"/World/{obstacle_name}",
                name=obstacle_name,
                position=position,
                scale=np.array([0.3, 0.3, 1.5]),
                color=np.array([0.7, 0.4, 0.2])
            )

def create_warehouse_env(local_assets=True):
    """创建仓库环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Warehouse")
        if prim is None:
            prim = define_prim("/World/Warehouse", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载仓库环境: {asset_path}")
    except Exception as e:
        print(f"无法加载仓库环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()

def create_warehouse_forklifts_env(local_assets=True):
    """创建叉车仓库环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Warehouse")
        if prim is None:
            prim = define_prim("/World/Warehouse", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载叉车仓库环境: {asset_path}")
    except Exception as e:
        print(f"无法加载叉车仓库环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()

def create_warehouse_shelves_env(local_assets=True):
    """创建货架仓库环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Warehouse")
        if prim is None:
            prim = define_prim("/World/Warehouse", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载货架仓库环境: {asset_path}")
    except Exception as e:
        print(f"无法加载货架仓库环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()

def create_full_warehouse_env(local_assets=True):
    """创建完整仓库环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Warehouse")
        if prim is None:
            prim = define_prim("/World/Warehouse", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载完整仓库环境: {asset_path}")
    except Exception as e:
        print(f"无法加载完整仓库环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()

def create_hospital_env(local_assets=True):
    """创建医院环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Hospital")
        if prim is None:
            prim = define_prim("/World/Hospital", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Hospital/hospital.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载医院环境: {asset_path}")
    except Exception as e:
        print(f"无法加载医院环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()

def create_office_env(local_assets=True):
    """创建办公室环境"""
    add_semantic_label()
    try:
        if local_assets:
            assets_root_path = "/home/xiaohuangfeng/datasets/isaac-assets/Assets/Isaac/4.5"
        else:
            assets_root_path = get_assets_root_path()
        
        prim = get_prim_at_path("/World/Office")
        if prim is None:
            prim = define_prim("/World/Office", "Xform")
        
        asset_path = assets_root_path + "/Isaac/Environments/Office/office.usd"
        prim.GetReferences().AddReference(asset_path)
        print(f"成功加载办公室环境: {asset_path}")
    except Exception as e:
        print(f"无法加载办公室环境，使用简单障碍物代替: {e}")
        create_simple_obstacles()