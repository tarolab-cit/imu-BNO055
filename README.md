# imu-BNO055

秋月電子 AE-BNO055-BO（Bosch BNO055 9軸センサフュージョンIMU）を **UART モード**で PC に接続し、
Python で定周期に読み出して表示・CSV 保存するプログラム。

- 製品ページ（取扱説明書 PDF もここから入手）: https://akizukidenshi.com/catalog/g/g116996/
- BNO055 データシート: https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/

## ハードウェア設定

- ジャンパーパッド: **J2 短絡、J3/J4/J5 開放**（PS1=0, PS0=1 → UART モード）
- 配線（UART-USB 変換との接続）
  | AE-BNO055-BO | UART-USB 変換 |
  |---|---|
  | 1 VIN | 3.3V または 5V |
  | 2 GND | GND |
  | 3 SDA/T (TxD) | RxD |
  | 4 SCL/R (RxD) | TxD |
- UART: 115200 bps, 8N1

## セットアップ

```
pip install -r requirements.txt
```

pyserial が必須。matplotlib は `--plot`（リアルタイムグラフ）を使うときだけ必要。

## 使い方

### 読み出し・表示・CSV 保存（imu_read.py）

```
python imu_read.py                            # COM5, 100 Hz, 表示のみ（全データを同じ位置で更新表示）
python imu_read.py --csv                      # imu_YYYYmmdd_HHMMSS.csv に保存
python imu_read.py --csv data/run1.csv        # ファイル名指定
python imu_read.py --duration 10 --csv        # 10 秒間記録して自動終了
python imu_read.py --rate 50                  # 50 Hz
python imu_read.py --calib bno055_calib.json  # 保存済み校正値を起動時に書き戻す
python imu_read.py --display-rate 0 --csv     # 画面表示なしで CSV 記録のみ
python imu_read.py --plot                     # 加速度・角速度・オイラー角をリアルタイムにグラフ表示
python imu_read.py --plot --plot-window 60    # グラフの時間幅を 60 秒に（既定 20 秒）
python imu_read.py --plot-mag                 # 地磁気（x/y/z と |B|）を別ウィンドウに表示
python imu_read.py --mode amg --acc-bw 7.81   # フュージョンなしの生データ、加速度の帯域幅を 7.81 Hz に
```

Ctrl+C で終了。動作モードは既定で NDOF（`--mode` で変更可、後述）で、BNO055 の全出力を取得する。サンプリングは `--rate`（既定 100 Hz）、画面表示は `--display-rate`（既定 10 Hz）で間引き。端末では全データを 10 行のブロックで同じ位置に更新表示し、`--scroll` またはパイプ時は 1 行ずつ出力する。

CSV の列:
`timestamp`(UNIX 時刻 [s]), `t`(開始からの経過 [s]),
`acc_x/y/z` [m/s²], `mag_x/y/z` [µT], `gyr_x/y/z` [deg/s],
`eul_heading/roll/pitch` [deg], `qua_w/x/y/z`, `lia_x/y/z`(線形加速度 [m/s²]), `grv_x/y/z`(重力 [m/s²]),
`temp` [°C], `calib_sys/gyr/acc/mag`(0〜3)

### リアルタイムグラフ（--plot）

`--plot` を付けると、加速度（x/y/z）・角速度（x/y/z）・オイラー角（roll/pitch/heading）を上から 3 段のグラフで
直近 `--plot-window` 秒ぶんスクロール表示する（matplotlib を使用）。横軸は 3 段で共通、縦軸は段ごとに自動スケール。
各段の右上に最新値を表示する。線の色は全段で共通で、青が x 軸まわり（x / roll）、橙が y 軸まわり（y / pitch）、青緑が z 軸まわり（z / heading）。

グラフの描画は別プロセスで行い、サンプリングループは測定値をキューに渡すだけなので、100 Hz の定周期と CSV 保存には影響しない。
記録が終わってもグラフウィンドウは残り、閉じるとプログラムが終了する。グラフを先に閉じても記録は続く。

```
python imu_read.py --plot --csv                        # 記録しながらグラフ表示
python imu_read.py --plot --calib bno055_calib.json    # 校正値を書き戻してグラフ表示
```

heading は 0〜360° で表されるため、北をまたぐと 360 から 0 に飛ぶ。これは異常ではない。
非フュージョンモード（`--mode amg` など）ではオイラー角が出ないため、下段は地磁気（x/y/z [µT]）になる。

### 地磁気の別ウィンドウ表示（--plot-mag）

`--plot-mag` を付けると、地磁気の x/y/z 成分と大きさ |B| = √(x²+y²+z²) を別ウィンドウに 2 段で表示する。`--plot` と併用できる。

```
python imu_read.py --plot-mag                 # 地磁気だけ
python imu_read.py --plot --plot-mag          # 姿勢のグラフと地磁気のグラフを 2 ウィンドウで
```

地磁気の外乱を観察する実演に使う。センサを回転させると x/y/z は大きく変わるが |B| はほぼ一定（日本では約 45〜50 µT）。
一方、磁石・鉄製品・スピーカー・PC を近づけると |B| が変わる。|B| が一定でないときは周囲の磁気環境が
地磁気を歪めており、NDOF モードの heading が信用できない状態だと分かる。

### 動作モードとセンサ設定（--mode, --acc-bw など）

BNO055 には「センサ内で姿勢を計算するフュージョンモード」と「9 軸の生データだけを出す非フュージョンモード」がある。

| `--mode` | 種別 | 出力 | 変更できるセンサ設定 |
|---|---|---|---|
| `ndof`（既定） | フュージョン | 全データ（オイラー角・クォータニオン・線形加速度・重力を含む） | 加速度レンジのみ |
| `imu` | フュージョン | 加速度 + ジャイロのみで姿勢計算（heading は相対値、地磁気不要） | 加速度レンジのみ |
| `amg` | 非フュージョン | 加速度・地磁気・角速度の生データ（オイラー角等は 0） | 下記すべて |
| `acconly` `gyronly` `accgyro` など | 非フュージョン | 選んだセンサだけ | 下記すべて |

フュージョンモードでは帯域幅などをセンサが自動で固定し、ユーザーの設定は無視される（モード切替時に既定値へ戻る）。
非フュージョンモードでは次のオプションで変更できる。帯域幅はセンサ内部のローパスフィルタのカットオフ周波数で、
狭くするほど滑らかになるが動きへの応答が遅れる。

| オプション | 値 | センサ既定 |
|---|---|---|
| `--acc-range` | 2, 4, 8, 16 [g] | 4 |
| `--acc-bw` | 7.81, 15.63, 31.25, 62.5, 125, 250, 500, 1000 [Hz] | 62.5 |
| `--gyr-range` | 125, 250, 500, 1000, 2000 [deg/s] | 2000 |
| `--gyr-bw` | 12, 23, 32, 47, 64, 116, 230, 523 [Hz] | 32 |
| `--mag-rate` | 2, 6, 8, 10, 15, 20, 25, 30 [Hz] | 10（フュージョン時は 20。直前にフュージョンモードで使っていた場合は `--reset` しない限り 20 のまま） |

```
python imu_read.py --mode amg --plot                              # 生データをグラフ表示（既定の帯域幅）
python imu_read.py --mode amg --acc-bw 7.81 --gyr-bw 12 --plot    # 帯域幅を最小に: 滑らかだが遅れる
python imu_read.py --mode amg --acc-bw 1000 --gyr-bw 523 --plot   # 帯域幅を最大に: 速いがノイズが多い
python imu_read.py --mode amg --acc-range 2 --plot                # レンジ 2 g: 強く振ると飽和する
```

実演の例: 同じ動き（センサを軽く振る、机を叩く）を `--acc-bw 7.81` と `--acc-bw 1000` で比べると、
フィルタで振動が消える代わりに立ち上がりが遅れることが加速度のグラフで分かる。
`--acc-range 2` で強く振ると ±19.6 m/s² で頭打ちになる（飽和）。
`ndof` と `amg` を比べると、フュージョンの便利さ（姿勢角がそのまま出る）と引き換えに内部設定が固定されることが分かる。

### キャリブレーション（imu_calibrate.py）

BNO055 はフュージョンモードで動作中に**自動で**校正します。専用コマンドはなく、センサを動かすだけです。
ただし校正結果は電源を切ると消えるため、このスクリプトで校正値（オフセット 22 バイト）を JSON に保存し、
次回 `imu_read.py --calib` で書き戻します。

```
python imu_calibrate.py                 # 校正状態を表示、全項目 3 になったら bno055_calib.json に保存
python imu_calibrate.py --save-now      # 完了を待たず現在値を保存
```

手順（画面の GYR/ACC/MAG/SYS が `[***]` になるまで）:

1. **ジャイロ**: 机の上に置いて数秒動かさない（すぐ 3 になる）
2. **加速度**: +X/-X/+Y/-Y/+Z/-Z の各面を上にして、それぞれ 2〜3 秒静置（6 面）。面を変えるときはゆっくり
3. **地磁気**: 空中で「8 の字」にゆっくり大きく動かす。PC・スピーカー・金属机から離す
4. **システム**: 上記が揃って少し静置すると 3 になる。NDOF モードの heading はここで正しくなる

## ファイル

- `bno055_uart.py` … BNO055 UART ドライバ（レジスタ定義、読み書き、`read_all()` 一括取得、校正値の読み書き、センサ設定の読み書き）
- `imu_read.py` … 定周期読み出し・表示・CSV 保存・グラフ表示（`--plot`）・動作モードとセンサ設定の指定
- `live_plot.py` … リアルタイムグラフ（`--plot` と `--plot-mag` の描画。別プロセスで matplotlib を描画。単体で `python live_plot.py` と実行すると疑似データでデモ）
- `imu_calibrate.py` … キャリブレーション補助・校正値保存

## 実測

`read_all()`（46 バイト一括読み出し）1 回あたり約 7〜10 ms。100 Hz でエラー・欠落なし（BNO055 のフュージョン出力レートも 100 Hz）。

## 注意

- BNO055 の UART は複数バイトの一括**書き込み**で `BUS_OVER_RUN_ERROR` を返すため、ドライバは 1 バイトずつ書き込む。読み出しは一括で問題ない。
- 地磁気が未校正（M0）のとき NDOF モードでは heading が 0 に固定され、姿勢角の更新も止まる。まず `imu_calibrate.py` で校正すること。
- 動作モード間の直接切替（例: AMG → NDOF）は `WRITE_FAIL` になることがあるため、ドライバの `set_mode()` は必ず CONFIG モードを経由する。
