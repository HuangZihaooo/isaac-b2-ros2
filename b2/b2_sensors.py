import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils

class SensorManager:
    """B2机器人传感器管理器"""
    
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        """添加RTX激光雷达传感器"""
        lidar_annotators = []
        for env_idx in range(self.num_envs):
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/lidar",
                parent=f"/World/envs/env_{env_idx}/B2/base_link",
                config="Hesai_XT32_SD10",
                translation=(0.2, 0, 0.2),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )

            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
            annotator.attach(hydra_texture.path)
            lidar_annotators.append(annotator)
        return lidar_annotators

    def add_camera(self, freq):
        """添加相机传感器"""
        cameras = []
        for env_idx in range(self.num_envs):
            camera = Camera(
                prim_path=f"/World/envs/env_{env_idx}/B2/base_link/front_cam",
                translation=np.array([0.4, 0.0, 0.2]),
                frequency=freq,
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
            camera.initialize()
            camera.set_focal_length(1.5)
            cameras.append(camera)
        return cameras