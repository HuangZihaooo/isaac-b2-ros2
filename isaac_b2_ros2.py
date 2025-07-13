from isaacsim import SimulationApp
import os
import hydra
import time

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")

@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    # 启动omniverse应用
    simulation_app = SimulationApp({
        "headless": False, 
        "anti_aliasing": cfg.sim_app.anti_aliasing,
        "width": cfg.sim_app.width, 
        "height": cfg.sim_app.height, 
        "hide_ui": cfg.sim_app.hide_ui
    })

    import omni
    import carb
    import b2.b2_ctrl as b2_ctrl
    from b2.b2_env import B2Environment, camera_follow
    import env.sim_env as sim_env

    # 初始化B2环境
    print("正在初始化B2环境...")
    b2_ctrl.init_base_vel_cmd(cfg.num_envs)
    
    # 创建B2环境
    env = B2Environment(num_envs=cfg.num_envs)
    env.setup_scene()

    # 仿真环境加载
    print(f"正在加载环境: {cfg.env_name}")
    if cfg.env_name == "obstacle-dense":
        sim_env.create_obstacle_dense_env()
    elif cfg.env_name == "obstacle-medium":
        sim_env.create_obstacle_medium_env()
    elif cfg.env_name == "obstacle-sparse":
        sim_env.create_obstacle_sparse_env()
    elif cfg.env_name == "warehouse":
        print(f"使用本地资源: {cfg.local_assets}")
        sim_env.create_warehouse_env(cfg.local_assets)
    elif cfg.env_name == "warehouse-forklifts":
        print(f"使用本地资源: {cfg.local_assets}")
        sim_env.create_warehouse_forklifts_env(cfg.local_assets)
    elif cfg.env_name == "warehouse-shelves":
        print(f"使用本地资源: {cfg.local_assets}")
        sim_env.create_warehouse_shelves_env(cfg.local_assets)
    elif cfg.env_name == "full-warehouse":
        print(f"使用本地资源: {cfg.local_assets}")
        sim_env.create_full_warehouse_env(cfg.local_assets)

    # 键盘控制（简化版）
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), 
        b2_ctrl.sub_keyboard_event
    )
    
    print("开始仿真...")
    
    # 重置环境
    env.reset()
    
    # 仿真循环
    frame_count = 0
    while simulation_app.is_running():
        # 步进仿真
        env.step()
        
        # 相机跟随（如果启用）
        if cfg.camera_follow:
            camera_follow(env)
        
        frame_count += 1
        if frame_count % 60 == 0:  # 每60帧打印一次状态
            states = env.get_robot_states()
            if states:
                pos = states[0]['position']
                print(f"帧数: {frame_count}, B2位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
    print("仿真结束")
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()