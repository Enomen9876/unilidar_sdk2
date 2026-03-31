import unilidar_py
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Extrinsics: IMU -> LiDAR
T_LI = np.array([
    [1.0,  0.0,  0.0, -0.007698],
    [0.0,  1.0,  0.0, -0.014655],
    [0.0,  0.0,  1.0,  0.006670],
    [0.0,  0.0,  0.0,  1.0]
])
R_LI = T_LI[:3, :3]

def main():
    lidar = unilidar_py.UnitreeLidar()
    if not lidar.initialize("192.168.1.62", "192.168.1.2", 6101, 6201):
        return

    lidar.start_rotation()
    lidar.set_work_mode(0)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Unitree L1 - World Fixed Frame", width=1280, height=720)
    
    pcd = o3d.geometry.PointCloud()
    # Osy teď představují fixní svět (ne LiDAR)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    
    grid = o3d.geometry.LineSet.create_from_triangle_mesh(
        o3d.geometry.TriangleMesh.create_box(width=20, height=20, depth=0.01)
    )
    grid.translate([-10, -10, -1.5]) 
    grid.paint_uniform_color([0.2, 0.2, 0.2])

    vis.add_geometry(axes); vis.add_geometry(grid); vis.add_geometry(pcd)
    vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])
    vis.get_render_option().point_size = 1.5

    cmap = plt.get_cmap("jet")
    
    # Globální rotace LiDARu vůči světu
    world_R_lidar = np.eye(3)
    
    print("Stabilizace: Obraz by měl nyní zůstat fixní vůči zemi.")

    try:
        while True:
            data = lidar.get_next_data()
            
            if data["type"] == "imu":
                q = data["quaternion"] # [w, x, y, z]
                try:
                    # 1. Převedeme kvaternion na matici (předpoklad Unitree [w,x,y,z])
                    # Pokud by to nefungovalo, zkuste R.from_quat([q[0], q[1], q[2], q[3]])
                    imu_R_world = R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
                    
                    # 2. Celková rotace LiDARu v prostoru
                    # Musíme započítat, jak je IMU v LiDARu namontováno
                    world_R_lidar = imu_R_world @ R_LI
                except:
                    pass

            elif data["type"] == "cloud":
                points_raw = data["points"]
                if len(points_raw) > 0:
                    # Body v lokálním systému LiDARu
                    pts_local = np.array([[p[0], p[1], p[2]] for p in points_raw])
                    
                    # 3. TRANSFORMACE DO SVĚTA:
                    # pts_world = pts_local @ world_R_lidar.T
                    # (Matice násobíme zprava transponovaně, což je v NumPy ekvivalent R * pts)
                    pts_world = pts_local @ world_R_lidar.T
                    
                    pcd.points = o3d.utility.Vector3dVector(pts_world)

                    # Barvení podle absolutní výšky ve světě
                    z_vals = pts_world[:, 2]
                    z_norm = np.clip((z_vals + 1.0) / 4.0, 0, 1)
                    pcd.colors = o3d.utility.Vector3dVector(cmap(z_norm)[:, :3])

                    vis.update_geometry(pcd)

            if not vis.poll_events(): break
            vis.update_renderer()
              
    except KeyboardInterrupt: pass
    finally:
        lidar.stop_rotation()
        vis.destroy_window()

if __name__ == "__main__":
    main()