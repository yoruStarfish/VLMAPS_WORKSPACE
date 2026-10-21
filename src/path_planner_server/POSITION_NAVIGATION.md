# move_in_between：自由終點朝向與全車身路徑限制

`robot.move_in_between('laptop', 'cardboard box')` 現在使用專用的
`PositionOnlyGrid`（`nav2_footprint_planner/StrictHybridPlanner`）。
普通導航繼續使用原本的 `GridBased` Smac Hybrid。

這次新增的是搜尋中的碰撞限制，不是把原本 Navfn 路徑檢查失敗後直接丟棄。
新規劃器以連續車身位姿進行 Hybrid A*，可選擇前進、倒車、左右圓弧，
並以 Reeds–Shepp 曲線嘗試連接自由朝向的終點。地圖、footprint、inflation、
MPPI 其他 critics、速度設定與 min_wz_filter 均未因此修改。

## 每次重新規劃的行為

1. 從 global costmap 在 mutex 內複製完整成本圖；讀取最新收到的 local
   `costmap_raw`，以 TF 轉入同一個規劃座標系。
2. 取用 global costmap 的實際 padded footprint（目前正方形 ±0.27 m）。
   目前 global/local footprint 設定相同；若日後更動，須保持兩者對車身的描述一致。
3. 搜尋每段候選移動時，檢查完整凸多邊形與 lethal 格的面積交集，包含
   車身內部、四角、格子邊界接觸，而非只看機器人中心或 footprint 周邊取樣。
4. 用相鄰車身多邊形的凸包，加上轉彎弧線偏離弦線的保守上界，檢查
   路徑點之間的整段掃掠區域。失敗的移動不進入搜尋佇列，繼續找別的路。
5. 找到路徑後，再取最新 global/local 快照，完整驗證輸出的路徑；
   不在驗證之後做平滑，以免平滑再次切入障礙物。

raw cost 254（lethal）是硬限制；目前 unknown 255 也是硬限制。
253 與其他膨脹成本不是自動判定整個 footprint 碰撞，仍以車身與 lethal
格是否相交判斷，並保留成本作為軟性路徑偏好。
local costmap 只約束它當下覆蓋的窗口；窗口之外由 global costmap 約束，
不宣稱已知道窗口外的即時障礙物。

搜尋以成本圖中心格的 Dijkstra 代價（距離＋global 軟成本）協助繞過大障礙物；這只引導搜尋，
不修改地圖，也不取代全車身碰撞檢查。規劃有解析度、朝向離散化與時間限制，
`TIMEOUT` / `NO_PATH` 並不構成連續空間中絕對無解的數學證明。

## 終點仍不要求朝向

- `FollowPosition` 複製目前 FollowPath MPPI 設定，只停用 GoalAngleCritic。
- `position_only_goal_checker` 繼承 XY 容許值（目前 0.18 m），yaw 容許值大於 π，stateful=false。
- `navigate_to_position.xml` 每秒重新規劃，明確選用這三個專用 ID。
- 終點 yaw 訊息是合法的佔位四元數；搜尋不以它限制到達角度。
  路徑中的車身方向仍有意義，因為轉彎會改變整車碰撞幾何。
- 路徑追蹤及失敗 recovery 仍可能轉向；這並非禁止所有旋轉。

普通 NavigateToPose / NavigateThroughPoses 的行為樹仍明確使用
`general_goal_checker`，一般 getPath 呼叫仍指定 `GridBased`。

## 參數與診斷

設定集中在 `config/planner_server.yaml` 的 `PositionOnlyGrid` 區段。
目前搜尋間距 0.0125 m、72 朝向 bins、最小轉彎半徑 0.2 m，
搜尋時間上限 1.0 s；輸出驗證間距 0.01 m 且有掃掠區域上界。
這些間距不會重採樣或修改原本的地圖。

`/PositionOnlyGrid/status`（std_msgs/String）及 planner_server 日誌會顯示：

- SUCCESS、搜尋節點數、被拒絕的邊數與搜尋時間。
- START_COLLISION、GOAL_COLLISION、TIMEOUT、NO_PATH、EXPANSION_LIMIT。
- LOCAL_COSTMAP_MISSING / STALE、LOCAL_TRANSFORM_STALE、GLOBAL_COSTMAP_NOT_CURRENT。
- MAP_CHANGED_PATH_INVALID：搜尋完成後的新成本圖使路徑失效。
- 幾何失敗時的 global/local、map-frame 車身位姿與對應成本圖 cell index。

`/PositionOnlyGrid/validated_path`（nav_msgs/Path）只發布成功驗證的路徑；
失敗時發布空路徑，避免 RViz 殘留上一次成功的 debug 路徑。
若啟用 ROS namespace，這兩個相對 topic 會加上 namespace；目前 local
costmap 訂閱是 YAML 中明確指定的 `/local_costmap/costmap_raw`。

local 成本圖缺失或 header 超過 1.0 s 不會被默認為無障礙；TF 缺失也會拒絕。
此設定對應目前 local costmap 發布頻率 2 Hz。搜尋前後都檢查新鮮度；
很慢的規劃或掉資料可能因此失敗，日誌會提供原因。

## 建置、載入與離線驗證

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-select nav2_footprint_planner path_planner_server \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /workspace/install/setup.bash
ctest --test-dir /workspace/build/nav2_footprint_planner --output-on-failure
PYTHONPATH=/workspace/src/path_planner_server PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m pytest src/path_planner_server/test/test_position_navigation.py -q -p no:cacheprovider
```

重啟原本的 Nav2 launch 及 VLMaps 程式後才會載入新插件。
本次修改沒有啟動、停止或重新啟動任何實車導航節點。

Ver13 離線重播：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
/usr/bin/python3 /workspace/src/nav2_footprint_planner/scripts/replay_ver13.py \
  /workspace/right_turn_debug_folder/right_turn_debug_ver13 \
  --executable /workspace/install/nav2_footprint_planner/lib/nav2_footprint_planner/footprint_replay \
  --output /workspace/right_turn_debug_folder/right_turn_debug_ver13/strict_planner_replay
```

僅對信任的本機 decoded.pkl 使用此工具；pickle 不是安全的外來資料格式。
工具不建立 ROS node、不發速度指令、不改成本圖；每個時間點保存 fixture、
CSV 路徑、JSON 結果。它另用獨立的正方形/格子 SAT 公式，以 ≤1 mm 位置
及 ≤0.002 rad 朝向間距取樣交叉驗證。此獨立檢查本身是密集取樣，
連續掃掠保證來自 C++ 的保守幾何上界。

## Ver13 重播可以、不能證明什麼

60、63、65、67、70、72.8 秒的實際起點都找到通過當時 global/local
lethal 幾何檢查的路徑；73.63 秒的實際起點已與 lethal 格相交，回傳
START_COLLISION。此時不會藉由忽略起點附近障礙來「脫困」。
應從進入窄處之前重新執行任務，驗證是否能避免走到原本的碰撞位置。

重播使用 Bag 中的 OccupancyGrid，而非未錄到的原始 raw 成本圖：
lethal/unknown 分類可重建，軟成本只能近似還原。線上插件直接使用 raw costs。
離線 global 快照有些距查詢時間 1–2 秒，local 約 0.15–0.57 秒；這是
有限的時間切片幾何實驗，不是 ROS 線上完整同步重播。

幾何可行不等於已證明實車可通過。部分路段在 padded footprint 外的
額外分離餘裕只有毫米等級；定位誤差、控制器偏離參考路徑及
min_wz_filter 對角速度的修改，仍可能讓實際車身越界。此修正沒有繞過
MPPI / collision monitor 的限制，也沒有保證一定有符合實際誤差的安全路徑。

下一次 Bag 請保留原本的 plan、cmd_vel 各階段、odom、TF、scan、footprint、
兩張成本圖，並增加 `/PositionOnlyGrid/status`、
`/PositionOnlyGrid/validated_path` 及兩張 `/costmap_raw`。比較是否成功到點、
首次 collision 的位置時間、實際車身相對新參考路徑的偏差，以及進度停滯時間。
不要為了這次測試同時調整 inflation、cost_scaling_factor 或 min_wz_filter。


## 搜尋效率修正 v1：只統一搜尋引導的成本尺度

本次只修改 `nav2_footprint_planner/src/core.cpp` 的搜尋引導，另增加一個
高成本區域繞障回歸測試。候選點產生與數量、72 個朝向、搜尋解析度、
1 秒時間上限、碰撞檢查、MPPI、inflation 與 min_wz_filter 都保持不變。
本次沒有加入 Smac 引導／區段修補，避免同時更換多個機制。

原本累積代價 g 包含距離與軟成本，估計剩餘代價 h 的 Dijkstra 卻只算距離。
長距離且具有膨脹成本的路線因此容易低估剩餘代價，在起點附近展開大量狀態。
現在 Dijkstra 相鄰格代價改為：

```
步長 × (1 + cost_penalty × 相鄰兩格平均軟成本 / 253)
```

這使用既有 cost_penalty=2.0，沒有新增參數或提高搜尋時間。
它只改變搜尋順序，不會把 lethal 變成可通過。到達條件另外保持使用
XY 歐氏距離，避免把加權代價當成位置容許誤差。搜尋不宣稱最短／最優路徑。

同一執行環境、相同 Ver13 輸入與 1 秒搜尋預算的離線比較：

| Ver13 時間 | 修改前 | 修改後 | 展開節點：前 → 後 |
|---|---|---|---|
| 30s，距終點約 5 m | TIMEOUT，1.000s | SUCCESS，約 0.021s | 25,087 → 576 |
| 40s | TIMEOUT，1.000s | SUCCESS，約 0.021s | 29,280 → 528 |
| 50s | SUCCESS，0.727s | SUCCESS，約 0.020s | 27,852 → 360 |
| 60/63/65/67/70/72.8s | 全部 SUCCESS | 全部 SUCCESS | 詳見比較資料 |
| 73.63s | START_COLLISION | START_COLLISION | 0 → 0 |

修改後九條成功路徑都通過原本的完整多邊形／掃掠檢查，以及獨立的密集
取樣碰撞交叉檢查。部分短程案例的節點數反而增加，並非每個起點都更快；
本次主要改善長距離搜尋。時間是本機離線測量，不是實車 CPU 的保證。

資料與最小程式補丁保存在：
`right_turn_debug_folder/right_turn_debug_ver13/search_efficiency_v1/`。
此目錄與舊 `strict_planner_replay` 分開，保留修改前後的證據。

本次現場逾時的 Bag 已被刪除且無法還原，因此尚未在該次完全相同的
起點／raw 成本圖上驗證。現場日誌仍可確認 24 次候選中 11 次 TIMEOUT、
13 次 GOAL_COLLISION；本次加速不會放寬終點幾何檢查，不能保證所有
GOAL_COLLISION 候選都變成可接受。下一次實驗應優先確認首個可行候選是否
能快速找到完整路徑，再確認能否通過瓶頸。無需同時調整其他導航參數。

建置兩個套件並重新載入工作區、重啟原本的 Nav2 後才會生效；
這份修改沒有自行重啟任何 ROS 節點或發出速度指令。

## 路徑沿用 v1：先檢查能否接回，再決定是否重搜

本輪新增一個規劃器行為：優先保留仍能從目前姿態安全接上的舊路徑。
原本的全域搜尋函式、碰撞限制、地圖、footprint、MPPI 與 min_wz_filter 均未修改。
一般 GridBased 導航不受影響。

`PositionOnlyGrid` 新增兩個設定：

```yaml
reuse_path_enabled: true
reuse_max_age: 3.0
```

這是本輪唯一的新行為開關。設為 false 並建置、重啟 Nav2，可回到原本每次重搜的行為。
3 秒是本輪保守的快取壽命測試值，不是已量測最佳值；它從「上次成功完整搜尋」計算，
每次沿用不會延長壽命。目的是避免合法但不適合跟隨的路線被永久保留，屆時仍會重新搜尋。

每次要求規劃時，仍讀取新鮮 global/local 快照並套用原有 TF／成本圖有效性檢查：

1. 目標 XY、座標框架須相同，且快取尚未過期；目標朝向仍然自由。
2. 僅在舊路徑前 0.50 m 內找進度匹配點，不跳到遠處、可能重疊的後段。
   若目前中心離匹配點超過 0.10 m，直接重搜。
3. 完整驗證匹配點後的舊路徑。已走過且被裁掉的路段不要求回去重走。
4. 從當前完整姿態嘗試接到舊路徑。銜接方向與舊路段一致，不能新增前後退切換，
   銜接點只在匹配後的同方向段內選擇。使用同方向 Dubins 圓弧；倒車以等價反向形式處理。
5. 銜接須符合原本最小轉彎半徑，長度最多 0.30 m；只嘗試匹配點前方 0.25 m 的短接。
   這些限制刻意保守，失敗不代表全空間無解，而是交回原搜尋器處理。
6. 銜接與剩餘路徑均做完整 footprint／連續掃掠驗證，輸出之前再取新快照驗證一次。
   位置接近但方向接不上，不能直接畫一條直線拉回去。

沿用檢查的協作式時間預算是 0.05 s；它不做第二次全域搜尋。
Ver15 離線檢查最大約 0.0012 s。若拒絕沿用，再使用原本 1 秒搜尋上限；
總回呼時間另包含成本圖複製、TF 與發布，不能把 1 秒稱作整個回呼的硬即時上限。

沿用只是短期保留幾何可行方案，不會優先尋找成本更低的新路，也不保證控制器跟得上。
最多保留 3 秒，避免無限延用。
準確而言，插件會在自身規劃／資料有效性失敗、deactivate、cleanup 時清除；
它沒有訂閱 FollowPath 結果，所以不會在控制器第一次單獨報錯時立即獲知。
後續規劃仍受當前姿態、最新地圖與 3 秒期限約束。TF 時間語意沿用原版，未在這輪另改。

### 診斷

既有 `/PositionOnlyGrid/status` 仍以 SUCCESS／錯誤狀態開頭，新增欄位：

```
SUCCESS ... decision=REUSE reuse_reason=VALID search_seconds=0 reuse_seconds=...
SUCCESS ... decision=REPLAN reuse_reason=CACHED_PATH_COLLISION ...
SUCCESS ... decision=REPLAN reuse_reason=NO_VALID_CONNECTION ...
SUCCESS ... decision=REPLAN reuse_reason=CACHE_EXPIRED ...
```

欄位先後順序以實際日誌為準。`search_seconds` 不包含沿用檢查；
`reuse_seconds` 是沿用幾何檢查時間；`cache_age` 是本次檢查時距上次新搜尋的時間。
沿用成功還有 nearest_index、join_index、connection_length；碰撞拒絕會記錄
reuse_grid、reuse_cell、reuse_pose。NO_VALID_CONNECTION 若沒有碰撞格，可能是短接長度、
曲率或同方向限制無法滿足，而非成本圖碰撞。

資料缺失、目前姿態碰撞、地圖更新後路徑失效仍會拒絕；不忽略 lethal／unknown。

### Ver15 先行檢查結果與限制

對相鄰兩次已錄製成功路徑，用新一次的起始姿態與當時 raw 快照，檢查是否能沿用上一條：

| 結果 | 次數 |
|---|---:|
| 通過沿用檢查 | 38 |
| 沒找到符合上述短接限制的連接 | 21 |
| 舊路徑剩餘部分碰撞 | 7 |
| 偏離匹配範圍 | 1 |

入口 74–105 s 的 15 個方向方案改變點中，只有 3 個通過，其餘為 11 個短接失敗、
1 個剩餘路徑碰撞。這個結果不支持把「不保留舊路」認定為入口猶豫的唯一主因。
本輪只保留真的通過檢查的機會，不強行凍結舊方案。

這是固定錄製輸入的一步可行性檢查，不是閉迴路模擬，也未套用三秒壽命後重跑實車。
38/67 不等於線上一定沿用的比例，3/15 也不是必定消除的切換次數。
快照與內部成本圖有時間差；未證明此策略能讓下一次實車必定成功。

離線資料：`right_turn_debug_folder/right_turn_debug_ver15/path_reuse_evaluation/`。

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
/usr/bin/python3 /workspace/src/nav2_footprint_planner/scripts/replay_reuse.py \
  /workspace/right_turn_debug_folder/right_turn_debug_ver15 \
  --executable /workspace/install/nav2_footprint_planner/lib/nav2_footprint_planner/footprint_replay \
  --output /tmp/ver15_reuse_check
```

需要可信的本機 decoded.pkl、raw 成本圖與 footprint，工具只讀 Bag 解碼資料。

下一輪保持 Ver15 的 low_vx_threshold=0.03、min_wz=0.8 及其餘參數，
比較入口停留時間、方向切換次數、REUSE／REPLAN 原因與任務結果。
不把手動前進算作自主導航成功。無需新增 Bag topic，既有 status／validated_path 已包含診斷。
