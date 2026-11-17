# Voice server

使用本機 faster-whisper 將英文語音轉成 prompt，發布到 `/user_command`。
現有 `llm_cmd.py` 打字入口可繼續使用；LLM 與導航仍由導航節點負責。
不需要 Google 憑證，不會將錄音送往雲端。

## 環境與模型

專用 Conda 環境名稱為 `voice_server`，Python 3.10 配合 ROS 2 Humble。
新機器可使用以下規格建立環境：

```bash
conda env create -f /workspace/voice_server/environment.yml
```

Dockerfile 使用 `docker/voice_server_environment.yml`，與上述規格保持一致。
Docker rebuild 會安裝辨識套件，但不會下載模型。

首次使用需連網下載英文 `small.en` 模型：

```bash
conda run --no-capture-output -n voice_server python -c \
  "from huggingface_hub import snapshot_download; snapshot_download('Systran/faster-whisper-small.en', local_dir='/workspace/models/faster-whisper-small.en', allow_patterns=['config.json', 'model.bin', 'tokenizer.json', 'vocabulary.*', 'preprocessor_config.json'])"
```

模型位於主機掛載的 `/workspace/models/`，不納入 Git。重建容器後保留此
掛載目錄即可沿用。節點只從本機模型目錄載入，不會自行下載；缺少模型時
會在啟動時報錯。正常辨識不需要網路。

預設使用 CPU INT8、4 個 CPU threads。GPU 可透過 `device:=cuda` 與
`compute_type:=float16` 選用，但需自行先完成相容的 CTranslate2、CUDA、
cuBLAS/cuDNN 配置；目前環境規格不包含 GPU 函式庫，也未宣稱 GPU 已驗證。

## 建置與啟動

```bash
source /opt/ros/humble/setup.bash
cd /workspace
/usr/bin/python3 -m colcon build --packages-select voice_server
source /workspace/install/setup.bash
ros2 launch voice_server voice_server.launch.py

# test
ros2 launch voice_server voice_server.launch.py \
  command_topic:=/voice_test_command 
```

launch 自動透過 `conda run` 使用語音環境，不需要先 activate。
`ros2 run` 的 console script 由建置時的 Python 決定，因此建議用 launch。
若找不到 Conda，可加 `conda_executable:=/opt/miniforge3/bin/conda`；環境名稱
也可透過 `conda_env:=voice_server` 指定。

等待模型載入並顯示 Ready，再從另一個已 source ROS 的終端開啟持續聆聽：

```bash
# start
ros2 service call /voice_server/start std_srvs/srv/Trigger '{}'

# stop
ros2 service call /voice_server/stop std_srvs/srv/Trigger '{}'
```

看到 `listening` 再說英文指令，例如 `Go to the chair.`。連續安靜 2.5 秒，
或達 15 秒錄音上限後停止，使用 VAD 過濾非語音片段並辨識，最後發布文字。
Start 的 success 代表開啟聆聽，不代表辨識或導航成功。重複 Start 不會建立第二個錄音流程。
模型只在啟動時載入一次；不是即時字幕，沒有 TTS 播報。

```bash
ros2 topic echo /voice_server/status
ros2 topic echo /voice_server/transcript
```

狀態為 `idle`、`listening`、`recognizing`、`published`、`no_speech`、`error`、`stopping`。
VAD 不保證避免所有誤辨識；辨識到的文字會直接發布，不另做使用者確認。
停止節點時會等待目前本機推論返回；本機辨識沒有原先的雲端 RPC timeout。

導航程式需先啟動並開始訂閱 `/user_command`，否則文字可能無人接收。
導航端尚無忙碌拒收或語音取消介面，請等任務結束再送下一個指令。
麥克風必須能由容器存取；預設使用 PyAudio 預設輸入裝置。

## 只測試辨識，不執行導航

```bash
ros2 launch voice_server voice_server.launch.py command_topic:=/voice_test_command
```

另一終端執行 `ros2 topic echo /voice_test_command`，並呼叫 Start service。

## 啟動參數

| 參數 | 預設 | 用途 |
| --- | --- | --- |
| command_topic | /user_command | prompt 發布目標 |
| model_path | /workspace/models/faster-whisper-small.en | 本機模型目錄 |
| language_code | en | 辨識語言；中文應用 zh，且需多語言模型 |
| device | cpu | cpu、cuda 或 auto |
| compute_type | int8 | CTranslate2 運算精度 |
| cpu_threads | 4 | CPU 推論執行緒數 |
| input_device_name | 空字串 | 輸入名稱的唯一子字串；設定時優先於 index |
| input_device_index | -1 | PyAudio 索引；-1 使用預設裝置 |
| max_record_seconds | 15.0 | 最長錄音時間 |
| silence_seconds | 2.5 | 開始說話後的無聲結束時間 |
| speech_start_timeout | 8.0 | 等待開始說話的時間（仍受總錄音上限限制） |
| silence_threshold | 100.0 | PCM 平均絕對音量門檻 |
| wakeword_enabled | false | 可選喚醒詞功能 |
| wakeword_model_path | 空字串 | 自備 ONNX 喚醒詞模型 |
| wakeword_threshold | 0.5 | 喚醒門檻 |

參數修改後需重啟。例如：

```bash
ros2 launch voice_server voice_server.launch.py input_device_index:=2 silence_seconds:=1.5
```

未來中文支援需另下載多語言模型（例如 `Systran/faster-whisper-small`），
再改 `model_path` 及 `language_code:=zh`；英文專用 small.en 不支援中文。

可選喚醒詞需另安裝 `openwakeword`、`onnxruntime` 並準備模型資源。
預設模式完全不需要喚醒詞模型。

## 麥克風取樣率相容性

節點會探測指定裝置支援的 PCM 格式。若不支援 16 kHz，會嘗試裝置預設
取樣率及 48/44.1 kHz，必要時改用雙聲道。錄音與喚醒詞共同使用連續重採樣，
提供 16 kHz 單聲道給辨識模型，不會把 48 kHz 錄音誤當成 16 kHz 播放。
終端會顯示實際輸入裝置與格式。重採樣使用 PyAV，不需系統 FFmpeg 指令。

錄音結束時會顯示 peak、max_mean 與音量門檻。若 peak=0，先檢查輸入裝置、
硬體靜音與系統輸入音量；降低門檻無法修復完全為零的收音。

## 持續聆聽：Start 一次，Stop 才結束

`/voice_server/start` 現在開啟持續模式。每句辨識完成（或無人說話逾時）後，
節點會自動重新進入 `listening`，不需要每句都呼叫 service。
辨識時會暫停收音，請等下一次 `listening` 再說話，這不是同步串流錄音。

停止語音輸入：

```bash
ros2 service call /voice_server/stop std_srvs/srv/Trigger '{}'
```

Stop 會停止錄音、丟棄尚未發布的辨識結果，並停用喚醒詞監聽。
若本機推論尚在執行，顯示 `stopping`，待推論返回後進入 `idle`；期間 Start
會回覆請稍候。回到 idle 後，可再次 Start。已發布的指令不會被撤回，導航不會
因 Stop service 自動取消。Stop 回覆後不會再發布舊的辨識結果。

若啟動時選擇喚醒詞模式，偵測到喚醒詞也會開啟持續聆聽；Stop 之後需用
Start service 重新開啟，或重啟節點恢復喚醒詞待機。

裝置 index 是 PortAudio 每次枚舉得到的編號，可能隨裝置可用性改變。
可用 `input_device_name:='hw:1,6'` 選擇目前機器的數位麥克風輸入；硬體位置
在不同機器也可能不同。僅在唯一的可用輸入符合名稱時才會開啟，不會靜默
改選另一個麥克風。若裝置被主機錄音軟體或其他節點占用，先釋放再 Start。
錄音／辨識遇到例外會暫停持續模式，修正後再次 Start，不會每秒重試刷錯誤。
