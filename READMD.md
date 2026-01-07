# 建築即是動物
 這個程式的功能是在將一個初始量體切割成多個部分，並透過遞迴演算法嘗試不同的變化方式（旋轉、翻轉、位移），最後以樹狀圖的形式視覺化呈現所有可能的組合，並自動標示出發生碰撞的失敗案例。
 
![messageImage_1767444639615](https://github.com/user-attachments/assets/704c7bd7-4819-4a33-b344-4170474541a8)


## 1.導入資料庫(Library)

    import trimesh
    import numpy as np
    from compas.datastructures import Mesh
    from compas_viewer import Viewer
    

 ## 2. 參數配置
 設定變化量、切割面、樹狀圖大小及切割後各個量體顏色。
 
    
    POSSIBLE_STATES = [
        {"type": "original", "val": 0}, 
        {"type": "translate", "val": 0.4},
        {"type": "rotate", "val": 45},
        {"type": "flip", "val": 0} 
    ]
    
    CUT_PLANES = [
        {"normal": [1.0, 0.0, 0.0], "origin": [0.0, 0.0, 0.0]}, 
        {"normal": [0.0, 1.0, 0.0], "origin": [0.0, 0.0, 0.0]} 
    ]
    
    LAYER_HEIGHT = 4.0      
    TOTAL_WIDTH = 256 * 2.5 
    
    FIXED_COLORS = [
        (255, 80, 80),   # 紅
        (50, 200, 80),   # 綠
        (80, 100, 255),  # 藍
        (255, 180, 0)    # 橘
    ]
    

## 3. 定義幾何運算

### 尋找操作基準線
在初始量體中抓一個邊作為操作的基準線。
#### 運作邏輯：
a.找到模型最低的 Z 軸座標（底座高度）。  
b.找出所有端點都落在這個最低高度的邊。  
c.在這些底部的邊中，選取最長的一條。

    
    def get_reference_edge(mesh):
        """自動尋找網格的操作軸"""
        if mesh is None or len(mesh.vertices) == 0: return [0,0,0], [1,0,0]
        vertices = mesh.vertices
        edges = mesh.edges_unique
        min_z = np.min(vertices[:, 2])
        bottom_edges = []
        for edge in edges:
            p1 = vertices[edge[0]]
            p2 = vertices[edge[1]]
            if np.abs(p1[2] - min_z) < 1e-4 and np.abs(p2[2] - min_z) < 1e-4:
                vec = p2 - p1
                length = np.linalg.norm(vec)
                bottom_edges.append((length, p1, vec))
        if not bottom_edges:
            return vertices[edges[0][0]], vertices[edges[0][1]] - vertices[edges[0][0]]
        bottom_edges.sort(key=lambda x: x[0], reverse=True)
        return bottom_edges[0][1], bottom_edges[0][2]

### 定義矩陣變換

將變化方式（旋轉、翻轉、位移）轉換為數學矩陣。
#### 運作邏輯：
rotate：以 get_reference_edge 找到的邊為旋轉軸，旋轉指定的角度。  
flip：強制旋轉 180 度（π），效果像是把零件翻過來。  
translate：延著基準邊的方向位移指定的距離。

    def apply_edge_action(mesh, action_type, value):
        """執行變換"""
        origin, vector = get_reference_edge(mesh)
        vector_unit = vector / np.linalg.norm(vector)
        
        if action_type == "rotate":
            matrix = trimesh.transformations.rotation_matrix(np.radians(value), direction=vector_unit, point=origin)
        elif action_type == "flip":
            matrix = trimesh.transformations.rotation_matrix(np.pi, direction=vector_unit, point=origin)
        elif action_type == "translate":
            matrix = trimesh.transformations.translation_matrix(vector_unit * value)
        else:
            matrix = np.eye(4)
            
        mesh.apply_transform(matrix)
        return mesh

### 定義多重平面切割
使用一組平面（Planes）將初始量體切開成小量體。
#### 運作邏輯：
遞迴式切割：拿第一個平面切開模型得到兩塊小量體，再拿第二個平面去切這兩塊得到四塊，依此類推。  
cap=True 參數：自動補上切割面，確保切完後的零件依然是密封的實體，否則後續的碰撞檢測會失效。
    
    def slice_all_meshes(initial_mesh, planes):
        """遞迴切割"""
        current_pieces = [initial_mesh]
        for plane in planes:
            next_gen = []
            normal = np.array(plane["normal"])
            origin = np.array(plane["origin"])
            for mesh in current_pieces:
                # slice_plane 需要 mapbox_earcut
                p_a = mesh.slice_plane(plane_origin=origin, plane_normal=normal, cap=True)
                p_b = mesh.slice_plane(plane_origin=origin, plane_normal=-normal, cap=True)
                if isinstance(p_a, trimesh.Trimesh) and len(p_a.faces) > 0: next_gen.append(p_a)
                if isinstance(p_b, trimesh.Trimesh) and len(p_b.faces) > 0: next_gen.append(p_b)
            current_pieces = next_gen
        return current_pieces
    

 ## 4. 定義碰撞檢測 
 ### 快速過濾
將切出來的小量體簡化為規則的長方體盒子，以快速篩選是否有碰撞。
#### 運作邏輯：
取出各個小量體在xyz軸上的最大最小值。  
將各軸最大最小值連起來形成快速過濾用的邊界(Bounding Box)。  
檢測各個快速過濾用的邊界是否有交織，有的話就進入下一階段的碰撞檢測，沒的話就過關。
    
    def is_aabb_overlapping(mesh_a, mesh_b):
        """檢查 Bounding Box 是否重疊"""
        min_a, max_a = mesh_a.bounds
        min_b, max_b = mesh_b.bounds
        
        if max_a[0] < min_b[0] or min_a[0] > max_b[0]: return False
        if max_a[1] < min_b[1] or min_a[1] > max_b[1]: return False
        if max_a[2] < min_b[2] or min_a[2] > max_b[2]: return False
        
        return True

### 進階檢測
快速過濾沒過關的再精準的測一次碰撞。
#### 運作邏輯：
防錯縮放：將測試物件暫時縮小 1%，避免剛好貼在一起的面被誤判為碰撞。  
精確點雲檢查：檢查兩個量體之間是否存在彼此的點在對方模型內部(雙向迴圈)，只要有任何一個點跑進去，就判定為發生碰撞。  
異常處理：避免前幾步驟的cap失效，如果有模型破洞就直接視為有碰撞，避免產出更多有問題的子代。
    
    def check_collision(target_mesh, other_meshes):
        """幾何碰撞檢測"""
        if not other_meshes: return False
    
        test_mesh = target_mesh.copy()
        center = test_mesh.centroid
        test_mesh.vertices -= center
        test_mesh.apply_scale(0.99) 
        test_mesh.vertices += center
        
        test_points = test_mesh.vertices
    
        for obstacle in other_meshes:
            if not is_aabb_overlapping(test_mesh, obstacle):
                continue
    
            try:
                # 雙向檢查
                if np.any(obstacle.contains(test_points)): return True 
                if np.any(test_mesh.contains(obstacle.vertices)): return True
            except Exception:
                return True
                
        return False
    
## 5. 定義樹狀圖生成
### 空間計算
導入先前設定的樹狀圖大小參數。
    
    def build_tree_recursive(current_meshes, depth, x_center, available_width, viewer, is_failed=False):
        """
        is_failed: 如果為 True，代表這一組模型發生了碰撞，將被畫成灰色並停止生長。
        """
        
        y_pos = -depth * LAYER_HEIGHT
        is_final_layer = (depth >= len(current_meshes))

### 視覺化顯示
將模型繪製到場景中。
#### 運作邏輯：
碰撞失敗 (if is_failed)：會顯示有透明度的灰色量體，方便看清碰撞原因。  
正常狀態 (else)：會顯示彩色，下一步要變化的小量體會是實色，其他則有透明度。
       
        # --- A. 顯示模型 ---
        for i, mesh in enumerate(current_meshes):
            display_mesh = mesh.copy()
            matrix_pos = trimesh.transformations.translation_matrix([x_center, y_pos, 0])
            display_mesh.apply_transform(matrix_pos)
            compas_mesh = Mesh.from_vertices_and_faces(display_mesh.vertices, display_mesh.faces)
            
            # 顏色邏輯修改：失敗者全灰
            if is_failed:
                c = (150, 150, 150) # 灰色 (Gray)
                opacity_val = 0.5   # 稍微透明一點，像幽靈一樣
                show_edges_val = True # 顯示邊框讓人看清楚怎麼撞的
            else:
                c = FIXED_COLORS[i % len(FIXED_COLORS)]
                if is_final_layer:
                    opacity_val = 1.0; show_edges_val = True
                else:
                    if i == depth: opacity_val = 1.0; show_edges_val = True
                    else: opacity_val = 0.3; show_edges_val = False
            
            unique_name = f"L{depth}_X{int(x_center*10)}_{i}_{'FAIL' if is_failed else 'OK'}"
            viewer.scene.add(compas_mesh, name=unique_name, facecolor=c, opacity=opacity_val, show_edges=show_edges_val)
### 遞迴終止
防止程式無限執行或產生無效路徑。
#### 運作邏輯：
自然結束：當深度 (depth) 已經處理完最後一個零件，代表這條路徑已經生成完畢，停止生長。  
碰撞截斷：當發生碰撞，則不再繼續生成下一代。

        # --- B. 終止條件 ---
        # 如果已經到底層，或者「已經失敗(碰撞)」，就不再生下一代了
        if is_final_layer or is_failed:
            return
### 分支衍生與碰撞傳遞
若未發生碰撞，程式會嘗試生成下一代。
#### 運作邏輯：   
執行變形：嘗試所有可能，以獨立沙盒操作，確保每一次嘗試都不會汙染到原始資料。  
碰撞檢測：在生成子節點之前，檢查剛移動過的零件是否撞到了其他靜止的零件，將結果存入 is_collision_happened。  
遞迴呼叫 (Next Step)：計算子節點在 X 軸的座標，讓樹狀圖左右展開。呼叫自己進入下一層 (depth + 1)，並將「是否發生碰撞」作為 is_failed 參數傳遞下去。

        # --- C. 產生下一層 ---
        num_branches = len(POSSIBLE_STATES)
        sub_width = available_width / num_branches
        
        for i, state in enumerate(POSSIBLE_STATES):
            next_meshes = [m.copy() for m in current_meshes]
            
            # 應用變形
            target_index = depth
            if state["type"] != "original":
                try:
                    next_meshes[target_index] = apply_edge_action(
                        next_meshes[target_index], state["type"], state["val"]
                    )
                except: pass 
            
            # 碰撞檢測
            moving_part = next_meshes[target_index]
            other_parts = [m for idx, m in enumerate(next_meshes) if idx != target_index]
            
            # 這裡改了！如果撞到，is_collision_happened 為 True
            is_collision_happened = check_collision(moving_part, other_parts)
    
            # 計算座標
            start_x = x_center - (available_width / 2)
            child_x = start_x + (i * sub_width) + (sub_width / 2)
            
            # 繼續遞迴，但把「是否碰撞」的狀態傳下去
            # 如果 is_collision_happened 是 True，下一層就會被畫成灰色，然後停止
            build_tree_recursive(next_meshes, depth + 1, child_x, sub_width, viewer, is_failed=is_collision_happened)
    
 
 # 6. 主程式

#### 運作邏輯：   
初始化視覺化引擎、建立初始量體、執行初始切割、啟動遞迴生成、渲染與顯示。  
    
    if __name__ == "__main__":
        viewer = Viewer()
    
        print("=== 程式開始 ===")
        
        cube = trimesh.creation.box(extents=[1.0, 1.0, 1.0])
        cube.fix_normals()
        
        try:
            base_parts = slice_all_meshes(cube, CUT_PLANES)
            print(f"-> 切割完成，產生 {len(base_parts)} 塊量體。")
        except ValueError as e:
            print("\n!!! 錯誤: 請輸入 pip install mapbox_earcut !!!\n")
            raise e
    
        print("-> 開始生成樹狀圖...")
        print("   (灰色模型代表發生碰撞的無效方案)")
        
        build_tree_recursive(base_parts, 0, 0, TOTAL_WIDTH, viewer)
    
        print("=== 生成完畢 ===")
        viewer.show()

# 檢討與未來展望

目前已經達成幾何遞迴生成與視覺化，但在計算效能與物理精確度上仍有改進空間。以下列出目前版本的限制與可能的改善方向：

### 1. 碰撞檢測機制
* **目前狀況**：採用手寫的 `AABB`（粗篩）搭配 `Vertex Containment`（頂點包含檢測）。
* **潛在問題**：
    * **漏判 (False Negatives)**：若兩個模型互相穿插，但各自的頂點剛好都沒有進入對方的內部（例如兩個薄片呈十字交叉），目前的算法會誤判為「無碰撞」。
    * **效能瓶頸**：隨著遞迴深度增加，物件數量呈指數成長，`O(N^2)` 的雙向迴圈檢查會顯著拖慢生成速度。
* **改善建議**：改用 **`trimesh.collision.CollisionManager`**（基於 FCL 庫）。支援更精確的 Mesh-to-Mesh 檢測，且運算速度遠快於 Python 迴圈。

### 2. 記憶體管理
* **目前狀況**：每一次遞迴都會執行 `mesh.copy()` 產生全新的網格物件並存入記憶體。
* **潛在問題**：當遞迴深度 (`depth`) 超過 5 或 6 層時，場景中可能會有數千個網格物件，易導致 記憶體耗盡 或 Viewer 卡頓。
* **改善建議**：導入 **幾何實例化 (Geometry Instancing)** 技術。
    * 只在記憶體中保存一份「基礎零件」的幾何資料。
    * 樹狀圖中的每一個節點僅紀錄 **「變換矩陣 (Transformation Matrix)」**。
    * 渲染時再動態繪製，可大幅降低 RAM 的佔用率。

### 3. 切割演算法
* **目前狀況**：依賴 `trimesh.slice_plane(cap=True)` 與 `mapbox_earcut`。
* **潛在問題**：當切割平面剛好經過模型的頂點或邊線時，偶爾會產生 **非流形幾何 (Non-manifold geometry)** 或極小的破碎面，導致後續的布林運算或體積計算出錯。
* **改善建議**：
    * 在切割前加入 **`mesh.merge_vertices()`** 進行預處理。
    * 實作錯誤重試機制：若切割失敗，微幅偏移切割平面 (例如 `+0.0001`) 再重試。

### 4. 參數設定
* **目前狀況**：切割平面 (`CUT_PLANES`) 與變形規則 (`POSSIBLE_STATES`) 直接寫死在程式碼中。
* **改善建議**：將參數抽離至 **`config.json`** 或 YAML 檔，無需修改程式碼即可快速測試不同的生長邏輯與切割方案。

# AI Process
### Prompt 1：  
哪一項ai最適合用於寫程式 我想要用python寫一個程式 跑圖形 我想達成rhino中建模的效果:將一個立方體上隨意畫一條線 分割那個立方體 可以如何進行 

### Prompt 2：
我目前進行的成果如下，但跑出的圖形是完整的立方體，並沒有出現切割後的狀態，我可以如何繼續，請幫我檢驗程式碼，目前沒有安裝外掛，如果需要使用外掛，請給我詳細的指引:  
     
    from compas.datastructures import Mesh
    from compas.geometry import Plane
    from compas_viewer import Viewer
    
    # ----------------------------------------------------------
    # 1. 手動建立一顆立方體（不依賴 Box / Frame）
    # ----------------------------------------------------------
    vertices = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1],
    ]
    
    faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7]
    ]
    
    mesh = Mesh.from_vertices_and_faces(vertices, faces)
    
    # ----------------------------------------------------------
    # 2. 建立你畫的切割線（兩個點）
    # ----------------------------------------------------------
    p1 = [0.2, -1, 0.3]
    p2 = [0.2,  1, 0.7]
    
    # 把線轉成切割平面
    plane_origin = [(p1[i] + p2[i]) / 2 for i in range(3)]
    plane_normal = [p2[i] - p1[i] for i in range(3)]
    plane = Plane(plane_origin, plane_normal)
    
    # ----------------------------------------------------------
    # 3. 分類 mesh 頂點（平面前 / 平面後）
    # ----------------------------------------------------------
    side_pos = []
    side_neg = []
    
    for v in mesh.vertices():
        xyz = mesh.vertex_coordinates(v)
        d = plane.distance_to_point(xyz)
        if d >= 0:
            side_pos.append(v)
        else:
            side_neg.append(v)
    
    # ----------------------------------------------------------
    # 4. 根據點分類，生成兩個子 mesh
    # ----------------------------------------------------------
    faces_pos = []
    faces_neg = []
    
    for f in mesh.faces():
        verts = mesh.face_vertices(f)
        if all(v in side_pos for v in verts):
            faces_pos.append(verts)
        elif all(v in side_neg for v in verts):
            faces_neg.append(verts)
    
    mesh_pos = Mesh.from_vertices_and_faces(
        [mesh.vertex_coordinates(v) for v in side_pos], faces_pos)
    
    mesh_neg = Mesh.from_vertices_and_faces(
        [mesh.vertex_coordinates(v) for v in side_neg], faces_neg)
    
    # ----------------------------------------------------------
    # 5. 顯示結果
    # ----------------------------------------------------------
    viewer = Viewer()
    viewer.scene.add(mesh_pos, name="Upper", facecolor=(255, 80, 80))
    viewer.scene.add(mesh_neg, name="Lower", facecolor=(80, 120, 255))
    viewer.show()

### Prompt 3：

    Error: compas_cgal is not installed or failed to import.
    
    Please follow the installation guide.
  
### Prompt 4：  
跑出error了但我的conda有安裝成功 只是下次如果要指示我進行相關步驟 請清楚地寫出我應該貼在哪個程式中 另外我不太清楚終端機、conda等等有何差異麻煩說明  

    try running `pip install mapbox-earcut manifold3d`or `triangle`, `mapbox_earcut`, then explicitly pass:
    
    `triangulate_polygon(*args, engine="triangle")`
    
    to use the non-FSF-approved-license triangle engine
    
    Traceback (most recent call last):
    
      File "c:\Users\DCCG\Practices\1210.py", line 20, in <module>
    
        mesh_a, mesh_b = cube.slice_plane(
    
      File "C:\Miniconda3\envs\DCCG\lib\site-packages\trimesh\base.py", line 2357, in slice_plane
    
        new_mesh = intersections.slice_mesh_plane(
    
      File "C:\Miniconda3\envs\DCCG\lib\site-packages\trimesh\intersections.py", line 786, in slice_mesh_plane
    
        vn, fn = triangulate_polygon(p, engine=engine, force_vertices=True)
    
      File "C:\Miniconda3\envs\DCCG\lib\site-packages\trimesh\creation.py", line 675, in triangulate_polygon
    
        raise ValueError("No available triangulation engine!")
    
    ValueError: No available triangulation engine!
    
    (DCCG) PS C:\Users\DCCG> pip install mapbox-earcut manifold3d
    
    Collecting mapbox-earcut
    
      Downloading mapbox_earcut-2.0.0-cp39-cp39-win_amd64.whl.metadata (2.5 kB)
    
    Collecting manifold3d

### Prompt 5：  

我希望最終成果可以利用compas視覺化，另外還是有錯誤ㄟ:  

    line 20, in <module>
    
        mesh_a, mesh_b = cube.slice_plane(
    
    TypeError: cannot unpack non-iterable Trimesh object

### Prompt 6：  

    line 62, in <module>
    
        viewer.scene.update()
    
    AttributeError: 'ViewerScene' object has no attribute 'update'

### Prompt 7：  

啊我希望切完之後兩個部分都保留 現在是只保留一個嗎  

### Prompt 8：  

只看到part Aㄟ part B不見了

### Prompt 9：  

還是沒出現part B 切完之後不要移動 幫我把兩塊分別定義不同顏色就好

### Prompt 10：  

現在正常了 你好棒 現在我希望其中一塊可以沿切面的任意方向平移 先假設綠色那塊在移動好了 

### Prompt 11：  

好棒 如果我改變切面的方向或位置 這個程式還能運作嗎

### Prompt 12：   

有點複雜 但總之就是平移的方向要平行切面方向 或垂直切面的法向量對吧 可以幫我把完整程式碼寫出來嗎

### Prompt 13：   

    line 23
    
        [Image of two perpendicular vectors N and T on a 3D plane]
    
               ^
    
    SyntaxError: invalid syntax

### Prompt 14：   

好棒喔 阿如果我要旋轉咧 以切面的法向量為軸旋轉

### Prompt 15：   

那我想要再複雜一點 我想用這個程式碼同時測試多種方案 比如說切這個線跟切那個線... 然後兩個方案各衍伸平移一單位、兩單位... 再各衍伸旋轉30度、60度、90度...

### Prompt 16：   

那如果我希望螢幕上一次顯示三組咧 可能每個結果隔一段距離顯示這樣




