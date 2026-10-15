# imu-BNO055

秋月電子 AE-BNO055-BO（Bosch BNO055 9 軸 IMU）を UART で PC につなぎ、Python で読み出して表示・CSV 保存・グラフ表示するプログラム。

- 製品ページ（取扱説明書 PDF あり）: https://akizukidenshi.com/catalog/g/g116996/
- BNO055 データシート: https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/

## 配線

- ジャンパー: **J2 短絡、J3/J4/J5 開放**（UART モード）
- UART-USB 変換器との接続（115200 bps, 8N1）

| AE-BNO055-BO | UART-USB 変換 |
|---|---|
| 1 VIN | 3.3V または 5V |
| 2 GND | GND |
| 3 SDA/T | RxD |
| 4 SCL/R | TxD |

## セットアップ

```
pip install -r requirements.txt
```

| OS | ポート名の例 | 備考 |
|---|---|---|
| Windows | `COM7` | デバイスマネージャーの「ポート (COM と LPT)」で確認 |
| Linux | `/dev/ttyUSB0` | `sudo usermod -aG dialout $USER` で権限付与。グラフ表示には `python3-tk` が必要 |
| macOS | `/dev/cu.usbserial-XXXX` | `ls /dev/cu.*` で確認 |
| WSL | `/dev/ttyUSB0` | [Install_WSL.md](Install_WSL.md) を参照 |

## 使い方

### 読み出し（imu_read.py）

```
python imu_read.py --port COM7                      # 表示のみ
python imu_read.py --port COM7 --csv                # imu_YYYYmmdd_HHMMSS.csv に保存
python imu_read.py --port COM7 --csv run1.csv       # ファイル名指定
python imu_read.py --port COM7 --duration 10 --csv  # 10 秒で自動終了
python imu_read.py --port COM7 --plot               # 加速度・角速度・オイラー角をリアルタイム表示
python imu_read.py --port COM7 --plot-mag           # 地磁気（x/y/z と |B|）を別ウィンドウに表示
python imu_read.py --port COM7 --calib bno055_calib.json   # 保存済み校正値を書き戻す
```

Ctrl+C で終了。サンプリングは 100 Hz（`--rate` で変更）。

CSV の列: `timestamp`(UNIX 時刻), `t`(経過秒), `acc_x/y/z`[m/s²], `mag_x/y/z`[µT], `gyr_x/y/z`[deg/s],
`eul_heading/roll/pitch`[deg], `qua_w/x/y/z`, `lia_x/y/z`(線形加速度), `grv_x/y/z`(重力), `temp`[°C], `calib_sys/gyr/acc/mag`(0〜3)

### キャリブレーション（imu_calibrate.py）

BNO055 は動作中に自動で校正するが、結果は電源を切ると消える。このスクリプトで校正値を JSON に保存し、次回 `--calib` で書き戻す。
**地磁気が未校正だと heading が 0 に固定される**ので、最初に一度は行うこと。

```
python imu_calibrate.py --port COM7      # 全項目が 3 になったら bno055_calib.json に保存
```

1. **ジャイロ**: 机に置いて数秒静止
2. **加速度**: 6 面（±X, ±Y, ±Z）をそれぞれ上にして 2〜3 秒静置
3. **地磁気**: 空中で 8 の字にゆっくり動かす（PC や金属から離す）

### 動作モードとセンサ設定

| `--mode` | 出力 |
|---|---|
| `ndof`（既定） | フュージョン。姿勢角・クォータニオンを含む全データ |
| `imu` | フュージョン。加速度 + ジャイロのみ（heading は相対値） |
| `amg` | 生データのみ。下記のセンサ設定が有効になる |

非フュージョンモード（`amg` など）でのみ指定可能:

| オプション | 値 |
|---|---|
| `--acc-range` | 2, 4, 8, 16 [g] |
| `--acc-bw` | 7.81〜1000 [Hz] |
| `--gyr-range` | 125〜2000 [deg/s] |
| `--gyr-bw` | 12〜523 [Hz] |
| `--mag-rate` | 2〜30 [Hz] |

```
python imu_read.py --mode amg --plot                              # 生データをグラフ表示（既定の帯域幅）
python imu_read.py --mode amg --acc-bw 7.81 --gyr-bw 12 --plot    # 帯域幅を最小に: 滑らかだが遅れる
python imu_read.py --mode amg --acc-bw 1000 --gyr-bw 523 --plot   # 帯域幅を最大に: 速いがノイズが多い
python imu_read.py --mode amg --acc-range 2 --plot                # レンジ 2 g: 強く振ると飽和する`
```
選べる値の一覧は `python imu_read.py -h` で確認できる。

## ファイル

- `imu_read.py` … 読み出し・表示・CSV 保存・グラフ表示
- `imu_calibrate.py` … 校正値の保存
- `bno055_uart.py` … BNO055 UART ドライバ
- `live_plot.py` … リアルタイムグラフ（別プロセスで描画）
