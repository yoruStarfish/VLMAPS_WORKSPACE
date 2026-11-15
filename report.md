# VLMap 地圖上的 Nav2 導航問題分析報告

更新日期：2026-10-22  
工作分支：`nav2/use_extra_point`  
範圍：現有 `right_turn_debug_ver1`～`right_turn_debug_ver22`、Nav2/VLMaps 設定、執行日誌與目前工作區程式。

## 1. 結論摘要

目前最主要的問題不是單一 MPPI 參數，也不是單純 `inflation_radius` 太大，而是 **VLMap 導航地圖中的狹窄走道比實際環境更窄，Nav2 以完整 robot footprint 檢查時，車身在固定區域反覆接觸或逼近 lethal obstacle**。

這項判斷有多組資料支持：

- Ver3、Ver9、Ver10、Ver12～Ver18 多次在相近走道位置失去前進能力。
- MPPI 在該區域反覆回報 `Optimizer fail to compute path`。
- Spin 與 BackUp recovery 隨後回報 `Collision Ahead`，所以 Nav2 連恢復動作也無法安全執行。
- 自訂 `PositionOnlyGrid` 在 Ver14、Ver16～Ver18 更直接回報 `START_COLLISION`、`FOOTPRINT_COLLISION` 或 `NO_PATH`。
- Ver3 的成本圖重建顯示，首次 MPPI 失敗時車身 footprint 邊緣已碰到 4 個 lethal cell；稍後 static map 也有 occupied cell 落入 footprint。
- 將 inflation radius 調小沒有消除原始 lethal cell，因此 Ver12、Ver13 仍在同一區域失敗。

`min_wz_filter`、目標朝向、Global/Local Costmap 差異、TF 延遲與 scan 延遲都會改變表現，但目前證據顯示它們多半是 **惡化失敗率或改變失敗形式的次要因素**，不是反覆卡在同一位置的唯一根因。

Ver17 曾在沒有人工介入的情況下，經 recovery 後成功通過；Ver15 也曾成功一次。這表示走道並非每次都幾何上完全不可通行，而是位於非常狹窄的可行邊界：小幅定位誤差、路徑偏移、角度、重規劃結果、動態 costmap 或 `min_wz_filter` 放大的旋轉，都可能讓 footprint 從勉強可行變成 collision。

## 2. 問題背景與現行導航架構

### 2.1 已知幾何條件

- 實際走道寬度：約 **1.30 m**。
- 車身 footprint：`0.50 m × 0.50 m` 的方形，座標為 `±0.25 m`。
- VLMap 建圖過程含 Gaussian filter、雜訊過濾與障礙物膨脹，因此地圖中的 free space 可能比實際世界窄。
- 目前不能直接修改既有 VLMap 地圖。

實體走道的理論側向餘裕是 `(1.30 - 0.50) / 2 = 0.40 m`。但這個數字不能直接代表 Nav2 可用餘裕，因為地圖障礙邊界、定位誤差、雷射障礙、costmap resolution 與 footprint 旋轉掃掠都會再吃掉空間。

### 2.2 目前工作區的重要設定

| 元件 | 目前設定 | 影響 |
|---|---:|---|
| Global planner | `SmacPlannerHybrid`，`REEDS_SHEPP` | 可規劃前進、後退與曲線路徑 |
| Global footprint | 方形 `±0.25 m`，padding `0.0005 m` | 規劃器應以完整車身判斷碰撞 |
| Global inflation | radius `0.25 m`，scaling `5.0` | 影響全域路徑成本分布 |
| Local footprint | 方形 `±0.25 m` | MPPI 以完整車身評估 |
| Local inflation | radius `0.40 m`，scaling `5.0` | 狹窄區域內的局部軌跡成本較高 |
| MPPI | `consider_footprint: true` | 不只檢查中心點，而是檢查車身 |
| MPPI CostCritic | `near_goal_distance: 1.0` | 接近目標 1 m 內改變 CostCritic 的作用範圍 |
| Progress checker | `0.10 m / 20 s` | 20 秒內不足 0.1 m 會視為無進度 |
| Goal tolerance | x/y `0.18 m`，yaw `0.18 rad` | 最終到點條件仍包含角度 |
| `min_wz_filter` | `min_wz: 0.6 rad/s` | 將小角速度放大至底盤可執行範圍 |

目前 global inflation radius `0.25 m` 略小於由 footprint 計算出的 inscribed radius 約 `0.251 m`，啟動時會出現警告。這是設定缺陷，但它不等於「機器人必然位於 lethal」，也不足以單獨解釋所有失敗。

## 3. 主要症狀

### 3.1 狹窄走道入口猶豫、前後切換與左右搖晃

機器人抵達入口附近後，規劃結果與 MPPI 的局部可行軌跡容易變動。Smac 可產生含短距離倒車的路徑；MPPI 也可能在左右兩側代價接近時交替選擇角速度。`min_wz_filter` 又會將很小的旋轉需求放大，形成肉眼明顯的轉向、前後切換或左右擺動。

### 3.2 走道內固定位置停止

這是最穩定、最重要的症狀。多個版本大多在 map `y ≈ 5.0～5.12`，`x` 約從入口附近到 `0.6 m` 的區域失敗。具體位置會因版本、定位與路徑略有不同，但不是隨機散布在整段導航路線。

### 3.3 Controller 失敗後 recovery 也失敗

典型序列是：

1. Global planner 曾產生路徑。
2. MPPI 回報 `Optimizer fail to compute path`。
3. Behavior Tree 嘗試 Spin 或 BackUp。
4. Recovery 的 footprint collision check 回報 `Collision Ahead`。
5. 等待後再規劃，仍可能回到相同狀態。
6. 最後 `Goal failed`，或使用者因長時間搖晃而取消。

因此「有 Global Path」不代表該路徑在最新 local costmap、當下車身姿態與 MPPI 動態限制下仍可執行。

### 3.4 TF 與感測資料延遲

多個版本出現：

- `Lookup would require extrapolation into the future`
- `Unable to transform robot pose into global plan's frame`
- Collision Monitor 因 scan 與目前時間相差約 1～2.5 秒而忽略 scan
- Planner/controller loop missed desired rate

TF 查詢失敗可直接中止某次 FollowPath，並觸發重新規劃或 recovery；scan 被忽略則代表那段時間 Collision Monitor 沒有使用雷射提供預期的即時碰撞保護。它們是實際問題，但因為沒有 TF 錯誤的版本也會在相同狹窄區域發生 footprint collision，所以不能將所有導航失敗都歸因於 TF。

## 4. Root Cause 分級

### 4.1 已由資料證實

#### A. 固定區域內存在 footprint 與 lethal/occupied cell 的幾何衝突

Ver3 在 Bag 起始後約 `60.74 s` 第一次出現 MPPI optimizer failure。重建當時姿態約為：

- map pose：`x=0.2288, y=5.0810, yaw=-0.0894 rad`
- local costmap 車身邊緣最高成本達 lethal，footprint 內有 4 個 lethal cell
- 約 `68 s` 時，static occupied cell 也落入 footprint

這與隨後 Spin、BackUp 都回報 `Collision Ahead` 一致。詳細原始分析見 [`right_turn_debug_ver3/evidence.txt`](right_turn_debug_folder/right_turn_debug_ver3/evidence.txt)。

#### B. MPPI 在關鍵區域無法持續產生有效控制

Ver3、Ver9、Ver10、Ver12、Ver13、Ver14、Ver16～Ver18 均出現 optimizer failure 或自訂規劃器的 footprint collision 狀態。這代表停止的直接原因主要發生於局部軌跡可行性或碰撞檢查階段，而不是底盤完全沒有執行正常速度命令。

#### C. Recovery 在碰撞狀態下無法保證脫困

Spin 與 BackUp 本身也檢查完整 footprint。當車身已被 costmap 判定碰撞，或旋轉/倒車的預測掃掠會碰撞時，recovery 會立即中止。這解釋了為何「多做幾次 recovery」並不能穩定解決問題。

#### D. 調小 inflation radius 無法消除 lethal obstacle

Inflation layer 只改變障礙物周圍的漸層成本與作用範圍；原始 static/obstacle layer 的 lethal cell 仍是 lethal。Ver12/Ver13 調小 inflation 後仍卡在相同區域，證明只調 inflation radius 不是根治方法。

### 4.2 高度可能

#### A. VLMap 的障礙表示把可行空間壓縮到臨界範圍

已知 VLMap 建圖含平滑及障礙物膨脹，且畫面中的走道比實體觀感更窄。這能同時解釋：

- 真車肉眼看似可通行，Nav2 卻以 footprint 判定碰撞。
- 不同版本反覆卡在相近位置。
- 某些 run 能勉強成功，另一些只差少量位置或角度便失敗。

尚未取得同一截面的「實際牆距、原始點雲邊界、VLMap lethal 邊界、定位誤差分布」四者精確對照，因此無法量化地圖到底多縮了多少。

#### B. Global 與 Local cost field 不一致，提高路徑難以追蹤的機率

目前 global inflation 是 `0.25 m`，local 是 `0.40 m`。兩者不必相等，但 global planner 可能接受一條成本較低或靠近障礙的路徑，而 local MPPI 在更寬的 local inflation 成本場中認為軌跡代價過高或沒有足夠餘裕。

這可解釋「Global Path 存在，但 Controller 無法走」的部分案例。不過在 footprint 已碰到 lethal 時，即使兩個 inflation 完全一致仍會失敗，所以它不是更底層的幾何根因。

#### C. `min_wz_filter` 放大小角度修正，讓方形 footprint 更容易掃到障礙

Ver3 資料顯示，Nav2 原始的小角速度在部分時段被放大到 `±0.8 rad/s`；Ver18 改成 `±0.6 rad/s` 後仍有類似情形。方形車身旋轉時的掃掠半徑比直行時寬，小幅路徑誤差可能因此演變成明顯轉向或碰撞判定。

此 filter 有底盤死區補償用途，不宜直接移除。Ver18 將補償降為 `0.6` 仍失敗，也證明它不是根因；它較像進入臨界區後的放大器。

### 4.3 伴隨問題，尚未證明是共同根因

- TF future extrapolation：能造成單次控制中止與突然轉向後停止，但不能解釋所有版本的固定位置失敗。
- scan 延遲：降低 Collision Monitor 的即時保護可靠度，必須修正，但沒有證據顯示它造成地圖上的 lethal cell。
- 自訂 `PositionOnlyGrid` 搜尋負載：可能造成 loop rate 與 TF 時序壓力；Ver19 支持需要調查負載關係，但未完成單一變因證明。
- 終點朝向：原先可能要求在狹窄區旋轉；取消特定朝向後 Ver13 仍卡在相同區域，因此不是主要根因。
- `near_goal_distance=1.0`：與 Ver10 搖晃減少同時發生，但目前沒有隔離資料能證明它直接造成卡住。

## 5. Ver1～Ver22 實驗整理

時間均為各 Bag 開始後的相對時間；「資料不足」表示檔案存在，但目前沒有足夠解析結果可下確定結論。

| 版本 | 主要變化或目的 | 結果與證據 | 判斷 |
|---|---|---|---|
| Ver1 | 初期基準 | Bag 存在，缺少整理後 evidence | 資料不足 |
| Ver2 | 初期調參 | 約 `91.32 s` Goal failed，recovery 有碰撞訊息 | 已出現 controller/recovery 失敗模式 |
| Ver3 | 早期版本，`min_wz` 約 `0.8` | `60.74 s` 首次 optimizer fail；footprint 內 4 個 lethal cell；約 `80.25 s` Goal failed | 首個具完整幾何證據的固定區域碰撞案例 |
| Ver4 | 參數迭代 | recovery collision；約 `170.77 s` optimizer fail，隨後 Goal failed | 仍未解決局部可行性 |
| Ver5 | 參數迭代 | 約 `115.14 s` optimizer fail，recovery collision | 仍未解決 |
| Ver6 | 參數迭代 | Bag 存在，缺少整理後 evidence | 資料不足 |
| Ver7 | 參數迭代 | Bag 存在，缺少整理後 evidence | 資料不足 |
| Ver8 | 參數迭代 | 有 TF future extrapolation 與 optimizer failure | 時序與局部規劃問題同時存在 |
| Ver9 | 使用者觀察為明顯左右搖晃 | 約 `77.96 s` 開始 optimizer failure，約 `96.29 s` Goal failed | 搖晃沒有避免最終失敗 |
| Ver10 | `near_goal_distance=1.0` | 搖晃減少，但約 `57.05 s` optimizer fail，約 `77.40 s` Goal failed | 改善表面穩定性，未解決幾何衝突 |
| Ver11 | global inflation 改為 `0.40 m` | 入口持續左右搖擺；最後由使用者取消；另有 TF/optimizer/recovery 異常 | 較高 global inflation 改變入口行為，但不是自動失敗結論 |
| Ver12 | 嘗試縮小 inflation；錄製時 local 可能仍是 `0.30 m` | optimizer/recovery collision，約 `99.47 s` 失敗 | inflation 測試值有不確定性；仍無法通過 |
| Ver13 | 不再要求 move-in-between 終點特定朝向 | 約 `73.63 s` 起 optimizer fail，recovery collision，約 `92.27 s` Goal failed | 排除「只有終點朝向」的假說 |
| Ver14 | 加入 footprint-aware `PositionOnlyGrid` 與搜尋效率修改 | 首次 `START_COLLISION` 約 `71.22 s`，多次 collision；約 `96.96 s` Goal failed | 自訂 plugin 把碰撞原因顯式化，未改變地圖幾何 |
| Ver15 | 加入或調整路徑 reuse | 錄製 run 成功；未錄製的下一次 run 失敗 | 結果不穩定，顯示系統處於臨界可行狀態 |
| Ver16 | 自訂 planner/reuse 實驗 | 11 次 `START_COLLISION`；入口附近 recovery collision，最後失敗 | 重用路徑仍無法保證脫離碰撞 |
| Ver17 | `reuse=false` | 約 `63～82 s` 卡住與 recovery，最後 Nav2 成功；使用者確認無人工介入 | 證明 recovery 偶爾可逃出，但不可保證 |
| Ver18 | `min_wz` 降至 `0.6` | 約 `84.3 s` 首次採樣碰撞；`105.85 s` optimizer fail；之後曾自行前進，最後約 `133.51 s` Goal failed | 降低補償仍未解決核心問題 |
| Ver19 | 短錄製，調查 TF 延遲 | 觀察到 TF extrapolation；懷疑重規劃/清成本圖/自訂搜尋負載 | 關聯存在，因果尚待單一變因驗證 |
| Ver20 | 固定入口 A，A 後改由 cmd_vel | Nav2 抵達 A，但未進入 A→B | localization node 未持續 spin、姿態更新/控制流程有缺陷 |
| Ver21 | 改善 A→B 流程 | 仍停在 A；旋轉實際狀態與程式判定不一致 | stale TF/odom 與同步控制流程仍不可靠 |
| Ver22 | 再修正轉向及新鮮 TF | 某次命令在導航前因 stale `map→base_link` TF 失敗；另一輪固定 A 任務被中止 | 尚不足以驗證完整 A→B 策略 |

## 6. 各項嘗試為何沒有根治

### 6.1 調整 Global/Local inflation radius

較小 radius 可以縮小高成本區，但不會刪除 static map 或 obstacle layer 產生的 lethal cell。若 footprint 已覆蓋 lethal，MPPI 與 recovery 仍會拒絕動作。

`cost_scaling_factor` 也只改變 inflation 區的成本衰減速度：數值加大通常使成本離開障礙後下降得更快，可能讓路徑更靠牆；它同樣不會把 lethal 改成可通行。因此單獨把 scaling factor 調大，無法保證解決目前問題，反而可能降低牆面餘裕。

### 6.2 改 `near_goal_distance`

Ver10 的搖晃比 Ver9 小，表示 Critic 啟用範圍可能改變了控制決策；但 Ver10 仍在走道中途失敗。現有資料只能證明行為相關，不能證明這個參數是卡住的 root cause。

### 6.3 改終點朝向

在狹窄區要求最後原地轉向確實可能使 footprint 掃到牆，因此取消固定朝向是合理修正。然而 Ver13 仍在相同位置失敗，表示機器人在抵達終點前就已碰到局部可行性限制。

### 6.4 自訂 footprint-aware planner

`PositionOnlyGrid` 會從當前成本圖搜尋候選路徑，並沿路檢查 footprint。它能提供 `START_COLLISION`、`GOAL_COLLISION`、`FOOTPRINT_COLLISION` 等清楚診斷，也曾加入 path reuse 以降低跳動。

它沒有改變成本圖本身，所以不能讓已被判定 collision 的起點變成合法。搜尋、驗證與反覆清成本圖也增加運算與時序複雜度。現在工作區已將這套 plugin 與專用 BT 移除，恢復單一 `GridBased/SmacPlannerHybrid`，以降低除錯變數。

### 6.5 cmd_vel 接管

曾測試兩種方向：

1. 偵測停滯或 `START_COLLISION` 後自動接管。
2. 對 `move_in_between('laptop', 'cardboard box')` 使用固定入口 A，再由 cmd_vel 沿 A→B 前進。

第一種做法因條件、取消 Nav2、TF、感測有效性與速度仲裁交互作用過多，造成一開始就停止或異常轉向，已回退。

第二種概念較單純，目前在 [`tracer_lang_robot.py`](vlmapsSrc/vlmaps/robot/tracer_lang_robot.py) 仍保留固定入口 `(-0.529, 5.066)` 的專用分支。但 Ver20～Ver22 顯示，A→B 的实现仍受 executor spin、新鮮 TF、轉向完成判定與速度發布流程影響，尚未完成可靠實車驗證。

cmd_vel 可以繞過 Nav2 global/local planner 的 lethal 拒絕，但不能讓障礙消失，也不應繞過實體碰撞保護。更嚴重的是，Collision Monitor 曾因 scan 延遲而忽略資料，因此目前還不能把「有 Collision Monitor」當成接管時永遠有效的安全保證。

## 7. VLMaps 語意導航層的其他問題

### 7.1 `move_in_between`

一般流程會找兩個物件的語意位置並生成中間目標。歷史問題包括：

- 強制指定終點 yaw，使車子可能在狹窄區額外旋轉。
- 候選點可能落在 footprint 不可行區。
- `laptop/cardboard box` 專用 A→B 分支需要可靠取得即時車身姿態。

### 7.2 `move_to_right` / `move_to_left`

原始做法主要依語意物件中心、邊界與相對方位生成單一左右目標，可能：

- 因影像/VLMap 座標與 ROS map 座標角度定義不一致而選錯方向。
- 在多個同類物件時得到空 `ids_list` 或 `None`，造成 `NoneType - float`。
- 目標中心點雖是 free，但完整 footprint 覆蓋 lethal。
- 只使用 bbox 半對角距離，離物件與地圖障礙的安全距離不足。

目前 `tracer_lang_robot.py` 已加入共用 `_move_to_object_side()`：選擇車前方較合理的物件、在左右方向產生多個候選點、用 global costmap 檢查完整 footprint，再用 GridBased 驗證可規劃性。這修正的是左右物件導航的目標選擇，與狹窄走道的固定 lethal 根因是兩個相關但不同的問題。

## 8. 目前程式與工作區狀態

以下是 2026-10-22 工作區狀態，不代表所有變更已提交：

- 分支：`nav2/use_extra_point`。
- 標準 Nav2 planner：`GridBased = SmacPlannerHybrid`。
- 自訂 `nav2_footprint_planner/PositionOnlyGrid` 原始碼、專用 BT 與相關測試目前在 Git working tree 中標記為刪除。
- `min_wz_filter` 仍啟用，`min_wz=0.6`。
- `move_in_between('laptop', 'cardboard box')` 仍有固定入口 A 與 A→B cmd_vel 的實驗分支。
- 左右物件導航已改為多候選、footprint-aware 的目標選擇。
- 工作區另有多個未提交修改與 log 變動；後續測試前應先保存 commit 或 patch，避免無法重建某一版本的確切狀態。

## 9. 建議的處理順序

### 優先 1：建立可重現的標準 Nav2 baseline

先固定目前 Smac + MPPI + `min_wz=0.6` 的設定，不再同時調 inflation、scaling、Critic 或 recovery。連續跑至少 3～5 次相同起點與相同語意目標，確認：

- 成功率。
- 第一次 optimizer failure 或 collision 的時間與 map pose。
- 失敗前 footprint 與 global/local raw costmap 的交集。
- TF age、scan age 與 controller loop rate。

這能判斷移除自訂 plugin 後，TF 延遲是否仍然發生，以及標準 Nav2 的基準失敗率。

### 優先 2：量化「地圖比實體窄」的程度

在重複失敗截面量測：

- 實體左右牆或障礙距離。
- static map 中兩側 lethal 邊界距離。
- local costmap 中即時障礙邊界距離。
- robot footprint、定位 95% 誤差與 costmap resolution。

只有完成這一步，才能判斷是否可安全使用較小 footprint、專用 costmap 表示或固定走道控制。直接猜 inflation/scaling 數值只會重複前面的試驗。

### 優先 3：若固定地圖不能改，選擇一個最小化的專用方案

可選方案依風險由低到高排列：

1. **固定 A 點後仍用 Nav2，但使用經量測驗證的走道專用 footprint/costmap。** 這仍保留規劃與感測避障，但需要 lifecycle-safe 的參數切換與恢復。
2. **固定 A 點後低速直行至 B。** 實作最短，但必須先解決 TF/odom 更新與 scan timestamp，並明確保留 Collision Monitor 或獨立急停。
3. **允許規劃器穿越 lethal。** 不建議。lethal 是碰撞語意，直接忽略會讓地圖中的真障礙與建圖假障礙無法區分。

若目標是短期示範且走道環境固定，方案 2 最容易隔離除錯；觸發條件應只綁定已知的 `laptop/cardboard box` 任務與固定 A，而不要重新引入全域「停滯就接管」狀態機。

### 優先 4：獨立修正 TF 與 scan 時序

即使採用固定 A→B，也應先確認：

- `map → odom → base_link` 持續更新。
- 控制端只使用足夠新鮮的 TF/odom。
- ROS clock、sensor stamp 與各主機時鐘一致。
- scan 不再被 Collision Monitor 以 1～2.5 秒延遲為由忽略。

這是安全與穩定性問題，不能用調 planner 參數取代。

## 10. 下一次 Bag 建議保留的資料

至少錄製：

- `/tf`、`/tf_static`
- `/odom`
- `/scan`
- `/plan` 或實際 global plan topic
- `/cmd_vel_nav`
- `/cmd_vel_smoothed`
- `/cmd_vel_filtered`
- `/cmd_vel`
- global/local footprint
- global/local costmap 與 `costmap_raw`
- `/rosout`
- NavigateToPose action 的 goal、feedback、result/status
- 若可用，MPPI optimal trajectory / candidate trajectories

每次測試另保存：

- 完整 `output_nav.log` 與應用程式 `output.log`。
- 測試當下 source 與 install space 的參數快照。
- Git commit hash、branch、`git diff`。
- 是否人工介入、取消時間與實體接觸情況。

建議比較指標：成功率、入口等待時間、第一次 collision pose、最小 footprint-to-lethal 距離、前後切換次數、角速度符號切換次數、TF/scan 最大 age、實際前進量與 progress checker 觸發時間。

## 11. 證據限制

- Ver1、Ver6、Ver7、Ver15、Ver19～Ver22 沒有與其他版本同等完整的 `evidence.txt`，部分結論來自 log 與當次觀察。
- 各版本不一定只改一個變因，且 source、install space、未提交工作區與實際啟動參數曾不一致。
- Ver12 錄製時 local inflation 可能為 `0.30 m`，不是原先假設的 `0.25 m`，因此不能當作「global/local 都精確為 0.25」的嚴格實驗。
- Ver11 由使用者手動取消，不能寫成 Nav2 自行宣告失敗。
- Ver15 第二次失敗沒有 Bag，僅能證明成功結果不可穩定重現。
- Ver17 與 Ver18 的後段移動經使用者確認沒有人工介入，可視為 Nav2/recovery 自行產生。
- 尚未取得失敗截面的精確實體幾何與地圖幾何對照，所以目前可以確定 Nav2 看見 collision，仍不能只靠 Bag 判定該 lethal cell 是真實障礙、地圖膨脹假象，或定位誤差造成。

## 12. 最終判斷

目前可以明確說：Nav2 不是「莫名其妙停住」。它在多次測試中一致地認為完整車身或預測軌跡會碰到 lethal/occupied 區域，因此 MPPI 停止，collision-aware recovery 也停止。造成這個判定反覆出現的最合理根因，是 VLMap 障礙表示、完整 footprint、定位/感測誤差與狹窄走道幾何共同形成的臨界可行空間。

單獨改 inflation radius、cost scaling、`near_goal_distance`、終點 yaw 或 `min_wz` 都不能把原始 lethal cell 變成安全 free space。後續應先以標準 Nav2 做可重現 baseline，再量化地圖與實體走道差異；若地圖確定不能修改，則使用範圍明確、程式簡短且可獨立驗證的走道專用策略，而不是再加入全域性的複雜接管狀態機。
