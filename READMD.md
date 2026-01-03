# 建築即是動物
 這個程式的功能是在將一個初始量體切割成多個部分，並透過遞迴演算法嘗試不同的變化方式（旋轉、翻轉、位移），最後以樹狀圖的形式視覺化呈現所有可能的組合，並自動標示出發生碰撞的失敗案例。


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

###進階檢測
快速過濾沒過關的再精準的測一次碰撞。
#### 運作邏輯：
防錯縮放：將測試物件暫時縮小 1%，避免剛好貼在一起的面被誤判為碰撞。  
精確點雲檢查：檢查兩個量體之間是否存在彼此的點在對方模型內部，只要有任何一個點跑進去，就判定為發生碰撞。  
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

    
    def build_tree_recursive(current_meshes, depth, x_center, available_width, viewer, is_failed=False):
        """
        is_failed: 如果為 True，代表這一組模型發生了碰撞，將被畫成灰色並停止生長。
        """
        
        y_pos = -depth * LAYER_HEIGHT
        is_final_layer = (depth >= len(current_meshes))
        
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
    
        # --- B. 終止條件 ---
        # 如果已經到底層，或者「已經失敗(碰撞)」，就不再生下一代了
        if is_final_layer or is_failed:
            return
    
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
    
 # ==========================================================
 # 5. 主程式
 # ==========================================================
    
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
