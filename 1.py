import trimesh
import numpy as np
from compas.datastructures import Mesh
from compas_viewer import Viewer
import time

# ==========================================================
# 參數配置與選擇區
# ==========================================================

CONFIGURATIONS = {
    # 方案 0: X 軸切 5 刀 (產生 6 塊)，像切吐司一樣
    0: {
        "NAME": "X-Axis Multi-Cut (5 cuts)",
        "NORMAL_VECTOR": [1.0, 0.0, 0.0],
        "START_POINT": [-0.4, 0.0, 0.0], # 切割的起始點 (方塊中心是 0,0,0，所以從 -0.4 開始切)
        "CUT_INTERVAL": 0.16,            # 每一刀的間隔距離
        "NUM_CUTS": 5,                   # 總共切幾刀
        "EXPLODE_DIST": 0.1,             # 展示時，每一片拉開的距離 (爆炸圖效果)
        "ROTATE_DEG": 0
    },
    
    # 方案 1: 斜角切 3 刀，並且旋轉每一片
    1: {
        "NAME": "45-Deg Multi-Cut (3 cuts)",
        "NORMAL_VECTOR": [1.0, 1.0, 0.0],
        "START_POINT": [-0.2, -0.2, 0.0],
        "CUT_INTERVAL": 0.2,
        "NUM_CUTS": 3,
        "EXPLODE_DIST": 0.3,
        "ROTATE_DEG": 15                 # 每一片都比上一片多轉 15 度
    }
}

TEST_INDICES = [0, 1] 
OFFSET_STEP = 2.5 

# ----------------------------------------------------------
# 核心邏輯：執行 N 次切割
# ----------------------------------------------------------
def slice_mesh_n_times(mesh, normal, start_origin, interval, num_cuts):
    """
    對網格進行多次切割，返回所有碎片的清單。
    邏輯：切下一片 -> 存起來 -> 剩下的部分繼續切
    """
    parts = []
    current_mesh = mesh
    
    # 確保法線是單位向量
    normal = np.array(normal)
    normal = normal / np.linalg.norm(normal)
    
    current_origin = np.array(start_origin)
    
    for i in range(num_cuts):
        # 如果剩下的網格已經空了，就停止
        if current_mesh is None or len(current_mesh.faces) == 0:
            break
            
        # 執行切割 (保留正向和負向)
        # 負向(Negative)通常是「刀子後面」那塊，我們把它當作切下來的薄片存起來
        # 正向(Positive)通常是「刀子前面」那塊，我們拿來下一輪繼續切
        # 注意：這邊的 Positive/Negative 取決於你的法線方向，如果切反了可以互換
        slice_result_keep = current_mesh.slice_plane(plane_origin=current_origin, plane_normal=-normal, cap=True)
        slice_result_remain = current_mesh.slice_plane(plane_origin=current_origin, plane_normal=normal, cap=True)
        
        # 1. 處理切下來的那一片 (存入清單)
        if isinstance(slice_result_keep, trimesh.Trimesh) and len(slice_result_keep.faces) > 0:
            parts.append(slice_result_keep)
        
        # 2. 更新「剩下的部分」，準備下一輪切割
        if isinstance(slice_result_remain, trimesh.Trimesh) and len(slice_result_remain.faces) > 0:
            current_mesh = slice_result_remain
            # 更新下一刀的切割原點 (往法線方向移動 interval 距離)
            current_origin = current_origin + (normal * interval)
        else:
            current_mesh = None # 切完了
            
    # 迴圈結束後，別忘了把最後剩下的那一塊也加進去
    if current_mesh is not None and len(current_mesh.faces) > 0:
        parts.append(current_mesh)
        
    return parts

# ----------------------------------------------------------
# 主執行迴圈
# ----------------------------------------------------------
viewer = Viewer()
total_offset_index = 0

for test_index in TEST_INDICES:
    config = CONFIGURATIONS.get(test_index)
    if not config: continue

    print(f"正在處理方案: {config['NAME']}...")

    # 1. 建立原始立方體
    cube = trimesh.creation.box(extents=[1.0, 1.0, 1.0]) 
    cube.fix_normals()

    # 2. 呼叫多重切割函式
    sliced_parts = slice_mesh_n_times(
        mesh=cube,
        normal=config["NORMAL_VECTOR"],
        start_origin=config["START_POINT"],
        interval=config["CUT_INTERVAL"],
        num_cuts=config["NUM_CUTS"]
    )

    # 3. 處理每一塊碎片 (移動、旋轉、顯示)
    # 為了讓結果分開顯示，我們使用 total_offset_index 計算整體 X 軸偏移
    base_x_offset = total_offset_index * OFFSET_STEP
    
    # 產生一組漸層顏色 (從紅到藍)
    for i, part in enumerate(sliced_parts):
        
        # --- 個別控制區 ---
        # 這裡示範如何讓每一塊「獨立移動」
        # 我們讓每一塊沿著法線方向炸開 (Explode)
        
        explode_vec = np.array(config["NORMAL_VECTOR"])
        explode_vec = explode_vec / np.linalg.norm(explode_vec)
        
        # 計算這一塊要移動多少 (索引 i 越大，移動越遠)
        move_dist = i * config["EXPLODE_DIST"]
        translation = explode_vec * move_dist
        
        # 應用「爆炸」移動
        matrix_explode = trimesh.transformations.translation_matrix(translation)
        part.apply_transform(matrix_explode)
        
        # 應用「個別旋轉」(如果有的話)
        if config["ROTATE_DEG"] != 0:
            angle = np.radians(config["ROTATE_DEG"] * i) # 每一塊轉的角度都增加
            # 繞著自己的中心轉，還是繞著原點轉？這裡是繞原點
            matrix_rot = trimesh.transformations.rotation_matrix(angle, [0,0,1])
            part.apply_transform(matrix_rot)

        # --- 整體顯示偏移 ---
        matrix_offset = trimesh.transformations.translation_matrix([base_x_offset, 0, 0])
        part.apply_transform(matrix_offset)
        
        # --- 轉為 Compas Mesh 並加入 Viewer ---
        compas_mesh = Mesh.from_vertices_and_faces(part.vertices, part.faces)
        
        # 根據索引給不同的顏色
        if i % 2 == 0:
            color = (255, 100, 100) # 紅色系
        else:
            color = (100, 100, 255) # 藍色系
            
        viewer.scene.add(compas_mesh, name=f"Set{test_index}_Part{i}", facecolor=color)

    total_offset_index += 1

viewer.show()