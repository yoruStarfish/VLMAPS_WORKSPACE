# 狀態：已退出執行鏈

已恢復固定角速度補償的 min_wz_filter 與原先 launch 參數；目前節點不再引用 RotationAssist 或 VelocityCompensator。以下保留先前實驗紀錄，參數表與新版行為均不代表目前執行設定。恢復原補償僅供基準比較，舊補償造成的低速左右放大仍存在。

# 有限時轉向起步補償

> 重新審查結果：目前版本尚不能作為走道實車測試的建議配置。
> 先前的測試只驗證限幅與抑制反覆放大，沒有驗證保留 ver3 的轉彎能力。
> ver3 的 53.524–54.422 s 區間，原 smoother 平均 wz=-0.0427 rad/s，
> 舊 filter=-0.8 rad/s；新版沿用錄製 odom 重播仍為 -0.0427 rad/s，
> 平均 vx=0.0737 m/s。這不能預測新版實車軌跡，卻顯示不能用此重播
> 證明有效補償被保留。停滯判定也無法辨別「有轉動但轉彎不足」。
> 下方實車流程暫緩；須先處理轉彎需求、實際追蹤誤差與前進速度的協調，
> 再做受控驗證，不能只憑反向次數下降直接進走道。

本次保留速度鏈：

```text
controller / behavior -> cmd_vel_nav
-> velocity_smoother -> cmd_vel_smoothed
-> min_wz_filter -> cmd_vel_filtered
-> collision_monitor -> cmd_vel -> 底盤
```

filter 內建角速度變化率限制，因此不用再重新接 topic。
沒有啟動任何實車程序。Source 已建置到 install。

## 為何修改

第七輪在 55–75 秒，filter 約 90% 訊息為 ±0.8 rad/s，並有 17 次大角度
左右切換；原始指令平均絕對值僅約 0.047 rad/s。
舊程式只憑低速就放大角速度，未確認車是否真的轉不動。

新版接入現有的 RotationAssist，使用 odom 判斷停滯，給有限時起步脈衝。
不再強制清除線速度，也不會因一個大角速度樣本就切成原地轉彎。
零角速度要求立即取消角補償；零速度停車與 command watchdog 不等待 slew。
缺少／過期 odom 時不提供補償，不延續先前補償的尾段。

## 目前 launch 參數（實驗起點）

| 參數 | 值 | 意義 |
|---|---:|---|
| min_wz | 0.8 | 保留已知可幫助轉動的目標值，非宣稱量到的死區 |
| trigger_wz | 0.01 | 更小的角速度需求不觸發補償 |
| low_vx_threshold | 0.08 | 僅在低線速度指令時考慮補償 |
| stopped_wz_threshold | 0.05 | odom 轉速幅度低於此值才累計停滯 |
| stall_time | 0.4 s | 同方向、符合條件的停滯需持續此時間 |
| pulse_duration | 0.2 s | 每次提高目標的時間上限 |
| cooldown | 1.0 s | 完整脈衝結束後的冷卻期 |
| max_angular_accel | 4.0 rad/s² | 以命令間隔限制輸出角速度變化，含退出尾段 |
| odom_timeout | 0.3 s | 按接收時間判斷回饋新鮮度 |
| command_timeout | 0.5 s | 命令中斷後清除狀態、watchdog 輸出零 |

反向會取消目前脈衝，先減少輸出，再對新方向重新累計停滯時間；不直接
把 +0.8 換成 -0.8。脈衝 0.2 s 是「補償目標提高」的上限；slew 下降
到原指令可能另有短暫尾段，因此不等於整段輸出高於原指令只有 0.2 s。

這些數值待實車驗證，不是安全認證或已辨識的底盤模型。角加速度限制為
命令限制，無法保證實體底盤的加速度。RotationAssist 是軟體補償，不能
代替速度低層閉迴路／模型校正，也不能修復地圖障礙邊界。

## 驗證

15 項測試通過（原有 RotationAssist 6 項，加新增補償器 9 項）：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/workspace/src/path_planner_server:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  /workspace/src/path_planner_server/test/test_rotation_assist.py \
  /workspace/src/path_planner_server/test/test_velocity_compensator.py
```

離線重播（已完成）：

```bash
PYTHONPATH=/workspace/src/path_planner_server:$PYTHONPATH /usr/bin/python3 \
  /workspace/right_turn_debug_ver7/replay_bounded_assistance.py
```

| 重播區間 | 舊 filter abs(wz)>=0.79 | 新版重播 |
|---|---:|---:|
| ver3 50–60s | 12.0% | 0% |
| ver4 61–71s（原本底盤停滯） | 98.0% | 6.5% |
| ver7 55–75s | 89.7% | 0.3% |

ver7 大角速度正負切換由 17 次降為 0 次。
ver4 的「舊 filter」是當時仍有發布但未送底盤的值；表格不是實車對照實驗。
所有重播沿用舊控制產生的 odom，僅檢驗命令變換，不能預測新的閉迴路
動作、碰撞距離或到點結果。ver3 原本可轉的段落新版很少補償，也可能因
新動作而出現不同回饋，故不能據此保證第三輪的轉彎能力維持不變。

## 實車下一步

完整停止舊導航程序後重新啟動；先在寬敞處檢查直行、小轉向與停止，
確認補償有需要時能啟動、移動後不持續放大，再測走道並錄成 ver8。
主要比較：方向反轉頻率、停滯時是否產生有效動作、補償是否造成過衝。
地圖 60–85 cm 與實際 130 cm 的邊界差異仍是獨立待處理問題。
