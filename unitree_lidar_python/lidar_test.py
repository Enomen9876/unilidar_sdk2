import unilidar_py
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def main():
    # 1. Inicializace LiDARu
    lidar = unilidar_py.UnitreeLidar()
    if not lidar.initialize("192.168.1.62", "192.168.1.2", 6101, 6201):
        print("Chyba: LiDAR nenalezen.")
        return

    lidar.start_rotation()
    lidar.set_work_mode(0)

    # 2. Nastavení Visualizeru
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Unitree L1 - RViz Mode", width=1280, height=720)
    
    # Nastavení vzhledu (RViz šedá)
    opt = vis.get_render_option()
    opt.background_color = np.array([0.15, 0.15, 0.15]) # Tmavě šedá
    opt.point_size = 2.0
    opt.light_on = False # Body budou svítit vlastní barvou

    # 3. Pomocné prvky (Mřížka a osy)
    pcd = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    
    # Vytvoření mřížky (Grid)
    grid = o3d.geometry.LineSet.create_from_triangle_mesh(
        o3d.geometry.TriangleMesh.create_box(width=10, height=10, depth=0.01)
    )
    grid.translate([-5, -5, -1.0]) # Posun mřížky pod LiDAR
    grid.paint_uniform_color([0.3, 0.3, 0.3])

    vis.add_geometry(axes)
    vis.add_geometry(grid)
    vis.add_geometry(pcd)

    # Barevná mapa (Jet)
    cmap = plt.get_cmap("jet")

    print("RViz vizualizace spuštěna...")
    first_batch = True

    try:
        while True:
            data = lidar.get_next_data()
            
            if data["type"] == "cloud":
                points_raw = data["points"]
                if len(points_raw) > 0:
                    # Převod na numpy
                    pts = np.array([[p[0], p[1], p[2]] for p in points_raw])
                    pcd.points = o3d.utility.Vector3dVector(pts)

                    # --- BARVENÍ PODLE VÝŠKY (Z) ---
                    z_values = pts[:, 2]
                    # Normalizace výšky (např. od -1m do 2m) pro barvy
                    z_min, z_max = -1.0, 2.5 
                    z_norm = (z_values - z_min) / (z_max - z_min)
                    z_norm = np.clip(z_norm, 0, 1) # Ořezání hodnot mimo rozsah
                    
                    colors = cmap(z_norm)[:, :3] # Aplikace colormapy (RGB)
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    # -------------------------------

                    vis.update_geometry(pcd)

                    if first_batch:
                        vis.reset_view_point(True)
                        # Nastavíme pohled trochu z nadhledu (jako v RViz)
                        ctr = vis.get_view_control()
                        ctr.set_zoom(0.5)
                        first_batch = False
            
            if not vis.poll_events():
                break
            vis.update_renderer()
              
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop_rotation()
        vis.destroy_window()

if __name__ == "__main__":
    main()