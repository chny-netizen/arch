import trimesh
import numpy as np
from compas.datastructures import Mesh
from compas_viewer import Viewer

# ==========================================================
# 參數配置與選擇區
# ==========================================================

# 1. 定義所有您想測試的方案
CONFIGURATIONS = {
    # 方案 0: 垂直 X 軸切割，平移 0.5 單位
    0: {
        "NAME": "X-Cut, Translate 0.5",
        "NORMAL_VECTOR": [1.0, 0.0, 0.0],
        "PLANE_ORIGIN": [0.0, 0.0, 0.0],
        "TRANSLATE": 0.5,
        "ROTATE_DEG": 0
    },
    
    # 方案 1: 45 度斜角切割，平移 1.0 單位
    1: {
        "NAME": "45-Deg Cut, Translate 1.0",
        "NORMAL_VECTOR": [1.0, 1.0, 0.0],
        "PLANE_ORIGIN": [0.0, 0.0, 0.0],
        "TRANSLATE": 1.0,
        "ROTATE_DEG": 0
    },

    # 方案 2: 垂直 Y 軸切割，旋轉 30 度
    2: {
        "NAME": "Y-Cut, Rotate 30 Deg",
        "NORMAL_VECTOR": [0.0, 1.0, 0.0],
        "PLANE_ORIGIN": [0.0, 0.0, 0.0],
        "TRANSLATE": 0.0,
        "ROTATE_DEG": 30
    },

    # 方案 3: 45 度斜角切割，旋轉 90 度
    3: {
        "NAME": "45-Deg Cut, Rotate 90 Deg",
        "NORMAL_VECTOR": [1.0, 1.0, 0.0],
        "PLANE_ORIGIN": [0.0, 0.0, 0.0],
        "TRANSLATE": 0.0,
        "ROTATE_DEG": 90
    }
}

# 2. 選擇您要同時顯示的方案索引列表
TEST_INDICES = [0, 1, 3] # <--- 這裡設定您想看的方案 (例如：顯示方案 0, 1, 和 3)

# 3. 定義每組方案之間的間隔距離
# 由於立方體大小為 1.0，我們沿 X 軸間隔 2.5 單位
OFFSET_STEP = 2.5 

# ----------------------------------------------------------
# 輔助函式：計算沿切面平移向量 (T)
# ----------------------------------------------------------
def calculate_translation_vector(normal_vec, distance):
    """計算一個與法線垂直的平移向量 (用於沿切面平移)。"""
    if distance == 0:
        return [0, 0, 0]
        
    normal_vec = np.array(normal_vec)
    N_unit = normal_vec / np.linalg.norm(normal_vec)
    
    # 找到一個與 N 垂直的向量 (使用 np.cross)
    # 參考向量選擇邏輯：避開與 N 平行的向量
    if np.abs(N_unit[0]) < 0.5 and np.abs(N_unit[1]) < 0.5:
        ref_vec = np.array([1, 0, 0])
    else:
        ref_vec = np.array([0, 1, 0])
        
    V_perp = np.cross(N_unit, ref_vec)
    
    # 如果 V_perp 仍然是零向量，換另一個參考向量
    if np.linalg.norm(V_perp) < 1e-6:
        ref_vec = np.array([0, 0, 1])
        V_perp = np.cross(N_unit, ref_vec)
        
    # 確保 V_perp 是單位向量並乘以距離
    V_perp_unit = V_perp / np.linalg.norm(V_perp)
    T = V_perp_unit * distance
    return T.tolist()

# ----------------------------------------------------------
# 主執行迴圈與視覺化區
# ----------------------------------------------------------
viewer = Viewer()
total_offset_index = 0

# 迴圈遍歷所有選定的測試方案
def new_func(PLANE_ORIGIN, cube, plane_normal_positive):
    result_a = cube.slice_plane(plane_origin=PLANE_ORIGIN, plane_normal=plane_normal_positive, cap=True)
    return result_a

for test_index in TEST_INDICES:
    config = CONFIGURATIONS.get(test_index)
    if not config:
        print(f"警告：方案 {test_index} 不存在，跳過。")
        continue

    # 載入參數
    NORMAL_VECTOR = np.array(config["NORMAL_VECTOR"])
    PLANE_ORIGIN = config["PLANE_ORIGIN"]
    TRANSLATE_DISTANCE = config["TRANSLATE"]
    ROTATION_DEGREES = config["ROTATE_DEG"]

    # 1. 建立並固定網格法線
    # 必須在迴圈內重新建立 cube，以確保每次操作都是在原始立方體上進行
    cube = trimesh.creation.box(extents=[1.0, 1.0, 1.0]) 
    cube.fix_normals()

    # 定義法線
    plane_normal_positive = NORMAL_VECTOR
    plane_normal_negative = [-x for x in NORMAL_VECTOR] 

    # 2. 執行雙重切割
    result_a = new_func(PLANE_ORIGIN, cube, plane_normal_positive)
    result_b = cube.slice_plane(plane_origin=PLANE_ORIGIN, plane_normal=plane_normal_negative, cap=True)

    # 3. 處理、轉換並應用方案內轉換 (平移/旋轉)
    mesh_list = []
    
    # 處理 Part A
    if isinstance(result_a, trimesh.Trimesh) and len(result_a.faces) > 0:
        mesh_a = result_a
        mesh_list.append((mesh_a, "Part A"))

    # 處理 Part B (應用方案內轉換)
    if isinstance(result_b, trimesh.Trimesh) and len(result_b.faces) > 0:
        mesh_b = result_b
        
        # 應用旋轉
        if ROTATION_DEGREES != 0:
            rotation_angle = np.radians(ROTATION_DEGREES)
            R_matrix = trimesh.transformations.rotation_matrix(
                angle=rotation_angle, direction=NORMAL_VECTOR, point=PLANE_ORIGIN
            )
            mesh_b.apply_transform(R_matrix)
        
        # 應用平移
        if TRANSLATE_DISTANCE != 0:
            translation_vector = calculate_translation_vector(NORMAL_VECTOR, TRANSLATE_DISTANCE)
            T_matrix = trimesh.transformations.translation_matrix(translation_vector)
            mesh_b.apply_transform(T_matrix)
            
        mesh_list.append((mesh_b, "Part B"))

    # 4. 應用並排顯示的【整體偏移】
    # 沿 X 軸移動，使多組結果分開
    x_offset = total_offset_index * OFFSET_STEP 
    
    if mesh_list:
        print(f"載入方案 {test_index}: {config['NAME']}，偏移量 X={x_offset}")

    for i, (mesh, part_name) in enumerate(mesh_list):
        # 應用整體平移矩陣
        OFFSET_MATRIX = trimesh.transformations.translation_matrix([x_offset, 0, 0])
        mesh.apply_transform(OFFSET_MATRIX)
        
        # 轉換為 compas Mesh 格式
        compas_mesh = Mesh.from_vertices_and_faces(mesh.vertices, mesh.faces)
        
        # 加入 Viewer (使用不同的顏色)
        color = (255, 80, 80) if part_name == "Part A" else (80, 255, 80)
        viewer.scene.add(compas_mesh, name=f"Set{test_index}_{part_name}", facecolor=color, opacity=0.8)

    total_offset_index += 1

# 顯示所有結果
viewer.show()
