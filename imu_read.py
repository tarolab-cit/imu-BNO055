"""
BNO055 (AE-BNO055-BO, UART モード) の全出力を定周期で読み込み、画面表示・CSV 保存するプログラム。

動作モードは既定で NDOF（9軸フュージョン）。BNO055 が出力する全データ
（加速度・地磁気・角速度・オイラー角・クォータニオン・線形加速度・重力・温度・校正状態）を取得する。
--mode amg にするとフュージョンなしの 9 軸生データになり、帯域幅（ローパスフィルタ）やレンジを自分で設定できる。

使い方:
    python imu_read.py                            # COM5, 100 Hz, 表示のみ
    python imu_read.py --csv                      # imu_YYYYmmdd_HHMMSS.csv に保存
    python imu_read.py --csv data/run1.csv        # ファイル名を指定して保存
    python imu_read.py --rate 50                  # 50 Hz
    python imu_read.py --display-rate 0 --csv     # 画面表示なしで CSV 記録のみ
    python imu_read.py --calib bno055_calib.json  # 保存済み校正値を起動時に書き戻す
    python imu_read.py --duration 10 --csv        # 10 秒間だけ記録して自動終了
    python imu_read.py --reset                    # 起動時にセンサをソフトリセット
    python imu_read.py --plot                     # 加速度・角速度・オイラー角をリアルタイムにグラフ表示（matplotlib）
    python imu_read.py --plot --plot-window 60    # グラフの時間幅を 60 秒に
    python imu_read.py --mode amg --plot          # フュージョンなし（生データ）。グラフ下段は地磁気になる
    python imu_read.py --mode amg --acc-bw 7.81 --gyr-bw 12 --plot   # 帯域幅を狭くして滑らかに（応答は遅れる）
    python imu_read.py --mode amg --acc-range 16 --gyr-range 250     # レンジ変更
    python imu_read.py --plot-mag                 # 地磁気（x/y/z と大きさ |B|）を別ウィンドウに表示（外乱の観察用）
    python imu_read.py --plot --plot-mag          # 両方

Ctrl+C で終了します。--plot / --plot-mag のときはグラフウィンドウが残るので、閉じると終了します。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

from bno055_uart import (BNO055UART, BNO055Error, IMUData, MODE_NAMES, is_fusion_mode,
                         ACC_RANGE_TABLE, ACC_BW_TABLE, GYR_RANGE_TABLE, GYR_BW_TABLE, MAG_RATE_TABLE)


def format_line(d: IMUData) -> str:
    """1 行表示（--scroll 指定時・パイプ／リダイレクト時用）"""
    return (
        f"t={d.t:8.3f}s | "
        f"acc[{d.acc_x:7.3f} {d.acc_y:7.3f} {d.acc_z:7.3f}] | "
        f"gyr[{d.gyr_x:8.3f} {d.gyr_y:8.3f} {d.gyr_z:8.3f}] | "
        f"mag[{d.mag_x:7.2f} {d.mag_y:7.2f} {d.mag_z:7.2f}] | "
        f"eul[{d.eul_heading:7.2f} {d.eul_roll:7.2f} {d.eul_pitch:7.2f}] | "
        f"q[{d.qua_w:6.3f} {d.qua_x:6.3f} {d.qua_y:6.3f} {d.qua_z:6.3f}] | "
        f"lia[{d.lia_x:7.3f} {d.lia_y:7.3f} {d.lia_z:7.3f}] | "
        f"grv[{d.grv_x:7.3f} {d.grv_y:7.3f} {d.grv_z:7.3f}] | "
        f"T={d.temp:3d} | calib S{d.calib_sys}G{d.calib_gyr}A{d.calib_acc}M{d.calib_mag}"
    )


BLOCK_LINES = 10


def _pad(label: str, width: int) -> str:
    """全角文字を幅 2 として label を width 桁に右パディング"""
    import unicodedata
    w = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in label)
    return label + " " * max(0, width - w)


def format_block(d: IMUData, n_ok: int, n_err: int) -> str:
    """端末用の複数行ブロック表示（BLOCK_LINES 行）"""
    def row(key: str, jp: str, unit: str, values: str) -> str:
        return f"{key:<5} {_pad(jp, 14)} [{unit:<5}]  {values}"

    lines = [
        f"t = {d.t:9.3f} s   samples = {n_ok:7d}   errors = {n_err:4d}",
        row("acc", "加速度", "m/s2", f"x={d.acc_x:8.3f}  y={d.acc_y:8.3f}  z={d.acc_z:8.3f}"),
        row("gyr", "角速度", "deg/s", f"x={d.gyr_x:8.3f}  y={d.gyr_y:8.3f}  z={d.gyr_z:8.3f}"),
        row("mag", "地磁気", "uT", f"x={d.mag_x:8.2f}  y={d.mag_y:8.2f}  z={d.mag_z:8.2f}"),
        row("eul", "オイラー角", "deg", f"heading={d.eul_heading:8.2f}  roll={d.eul_roll:8.2f}  pitch={d.eul_pitch:8.2f}"),
        row("qua", "クォータニオン", "-", f"w={d.qua_w:7.4f}  x={d.qua_x:7.4f}  y={d.qua_y:7.4f}  z={d.qua_z:7.4f}"),
        row("lia", "線形加速度", "m/s2", f"x={d.lia_x:8.3f}  y={d.lia_y:8.3f}  z={d.lia_z:8.3f}"),
        row("grv", "重力", "m/s2", f"x={d.grv_x:8.3f}  y={d.grv_y:8.3f}  z={d.grv_z:8.3f}"),
        row("temp", "温度", "degC", f"{d.temp:4d}"),
        row("calib", "校正状態 (0-3)", "-", f"sys={d.calib_sys}  gyr={d.calib_gyr}  acc={d.calib_acc}  mag={d.calib_mag}"),
    ]
    assert len(lines) == BLOCK_LINES
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="BNO055 UART 定周期読み出し・表示・CSV 保存")
    ap.add_argument("--port", default="COM5", help="シリアルポート (default: COM5)")
    ap.add_argument("--baud", type=int, default=115200, help="ボーレート (default: 115200)")
    ap.add_argument("--rate", type=float, default=100.0, help="サンプリング周期 [Hz] (default: 100)")
    ap.add_argument("--csv", nargs="?", const="", default=None, metavar="FILE",
                    help="CSV 保存。ファイル名省略時は imu_YYYYmmdd_HHMMSS.csv")
    ap.add_argument("--display-rate", type=float, default=10.0,
                    help="画面表示の更新頻度 [Hz] (default: 10)。0 で表示なし")
    ap.add_argument("--scroll", action="store_true",
                    help="表示を同じ位置で更新せず 1 行ずつスクロール出力する")
    ap.add_argument("--duration", type=float, default=None, help="記録時間 [s]。省略時は Ctrl+C まで")
    ap.add_argument("--calib", metavar="JSON", help="保存済み校正値 (imu_calibrate.py で作成) を起動時に書き戻す")
    ap.add_argument("--reset", action="store_true", help="起動時にソフトリセットする")
    mode_names = [m for m in MODE_NAMES if m != "config"]
    ap.add_argument("--mode", default="ndof", choices=mode_names,
                    help="動作モード (default: ndof)。amg はフュージョンなしの 9 軸生データ")
    ap.add_argument("--acc-range", type=int, choices=sorted(ACC_RANGE_TABLE), metavar="G",
                    help=f"加速度レンジ [g] {sorted(ACC_RANGE_TABLE)} (センサ既定 4)")
    ap.add_argument("--acc-bw", type=float, choices=sorted(ACC_BW_TABLE), metavar="HZ",
                    help=f"加速度の帯域幅 [Hz] {sorted(ACC_BW_TABLE)} (センサ既定 62.5)。非フュージョンモードのみ有効")
    ap.add_argument("--gyr-range", type=int, choices=sorted(GYR_RANGE_TABLE), metavar="DPS",
                    help=f"ジャイロレンジ [deg/s] {sorted(GYR_RANGE_TABLE)} (センサ既定 2000)。非フュージョンモードのみ有効")
    ap.add_argument("--gyr-bw", type=int, choices=sorted(GYR_BW_TABLE), metavar="HZ",
                    help=f"ジャイロの帯域幅 [Hz] {sorted(GYR_BW_TABLE)} (センサ既定 32)。非フュージョンモードのみ有効")
    ap.add_argument("--mag-rate", type=int, choices=sorted(MAG_RATE_TABLE), metavar="HZ",
                    help=f"地磁気の出力レート [Hz] {sorted(MAG_RATE_TABLE)} (センサ既定 10、フュージョン時 20)。非フュージョンモードのみ有効")
    ap.add_argument("--plot", action="store_true",
                    help="加速度・角速度・オイラー角をリアルタイムにグラフ表示する（matplotlib が必要）")
    ap.add_argument("--plot-mag", action="store_true",
                    help="地磁気の x/y/z と大きさ |B| を別ウィンドウにグラフ表示する（外乱の観察用）")
    ap.add_argument("--plot-window", type=float, default=20.0, metavar="SEC",
                    help="グラフに表示する時間幅 [s] (default: 20)")
    args = ap.parse_args()

    if args.plot or args.plot_mag:
        try:
            import matplotlib  # noqa: F401  存在確認のみ（描画は子プロセスで行う）
        except ImportError:
            print("--plot には matplotlib が必要です: pip install matplotlib", file=sys.stderr)
            return 1

    period = 1.0 / args.rate
    display_every = max(1, int(round(args.rate / args.display_rate))) if args.display_rate > 0 else 0

    try:
        imu = BNO055UART(args.port, args.baud)
    except Exception as e:
        print(f"シリアルポート {args.port} を開けません: {e}", file=sys.stderr)
        return 1

    mode = MODE_NAMES[args.mode]
    fusion = is_fusion_mode(mode)
    sensor_cfg = dict(acc_range=args.acc_range, acc_bw=args.acc_bw, gyr_range=args.gyr_range,
                      gyr_bw=args.gyr_bw, mag_rate=args.mag_rate)
    if fusion and any(v is not None for k, v in sensor_cfg.items() if k != "acc_range"):
        print(f"警告: フュージョンモード ({args.mode}) では帯域幅・ジャイロレンジ・地磁気レートの設定は無視されます。"
              "--mode amg などの非フュージョンモードで使ってください。", file=sys.stderr)

    with imu:
        try:
            imu.begin(mode, reset=args.reset)
            if any(v is not None for v in sensor_cfg.values()):
                imu.set_sensor_config(**sensor_cfg)
        except BNO055Error as e:
            print(f"BNO055 初期化失敗: {e}", file=sys.stderr)
            print("ジャンパー設定（UART: J2 短絡, J3/J4/J5 開放）と TX/RX 配線を確認してください。", file=sys.stderr)
            return 1

        if args.calib:
            with open(args.calib, encoding="utf-8") as f:
                offsets = bytes.fromhex(json.load(f)["offsets_hex"])
            imu.write_calibration_offsets(offsets)
            print(f"校正値を書き戻しました: {args.calib}")

        sys_status, st_result, sys_err = imu.get_system_status()
        cfg = imu.get_sensor_config()
        print(f"BNO055 接続OK  port={args.port}  mode={args.mode.upper()}  rate={args.rate} Hz")
        print(f"SYS_STATUS=0x{sys_status:02X}  ST_RESULT=0x{st_result:02X}  SYS_ERR=0x{sys_err:02X}")
        print(f"センサ設定: acc {cfg['acc_range']} g / {cfg['acc_bw']} Hz,  gyr {cfg['gyr_range']} dps / {cfg['gyr_bw']} Hz,  "
              f"mag {cfg['mag_rate']} Hz" + ("" if fusion else "  (非フュージョン: オイラー角等は 0 になります)"))

        # CSV 準備
        csv_file = None
        writer = None
        if args.csv is not None:
            path = args.csv or datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv")
            csv_file = open(path, "w", newline="", encoding="utf-8")
            writer = csv.writer(csv_file)
            writer.writerow(["timestamp"] + IMUData.field_names())
            print(f"CSV 保存先: {path}")

        # グラフ（別プロセス）。サンプリングループは Queue に入れるだけで描画を待たない
        plots: list[tuple] = []   # (Process, Queue) のリスト
        plot_q = mag_q = None
        if args.plot or args.plot_mag:
            from multiprocessing import Process, Queue
            from live_plot import run_plot, run_mag_plot
            title = f"BNO055 {args.mode.upper()}"
        if args.plot:
            plot_q = Queue()
            layout = "fusion" if fusion else "raw"   # 非フュージョンではオイラー角が出ないので下段を地磁気に
            p = Process(target=run_plot, args=(plot_q, args.plot_window, title, layout), daemon=True)
            p.start()
            plots.append((p, plot_q))
            print(f"グラフ表示: 直近 {args.plot_window:g} 秒、下段は{'オイラー角' if fusion else '地磁気'}"
                  "（ウィンドウを閉じても記録は続きます）")
        if args.plot_mag:
            mag_q = Queue()
            p = Process(target=run_mag_plot, args=(mag_q, args.plot_window, title), daemon=True)
            p.start()
            plots.append((p, mag_q))
            print(f"地磁気グラフ表示: 直近 {args.plot_window:g} 秒（x/y/z と |B|）")

        print("Ctrl+C で終了\n")

        # 端末に直接出力しているときは複数行ブロックを同じ位置で更新、それ以外は 1 行ずつ出力
        block_mode = bool(display_every) and not args.scroll and sys.stdout.isatty()
        if block_mode:
            os.system("")  # Windows コンソールで ANSI エスケープを有効化
            print("\n" * (BLOCK_LINES - 1))

        n_ok = n_err = 0
        t_start = time.perf_counter()
        next_t = t_start
        try:
            while True:
                next_t += period
                try:
                    d = imu.read_all()
                    n_ok += 1
                    if writer:
                        writer.writerow([f"{time.time():.6f}"] + d.as_list())
                    if plot_q is not None and plots[0][0].is_alive():
                        bottom = (d.eul_roll, d.eul_pitch, d.eul_heading) if fusion else (d.mag_x, d.mag_y, d.mag_z)
                        plot_q.put((d.t, d.acc_x, d.acc_y, d.acc_z, d.gyr_x, d.gyr_y, d.gyr_z, *bottom))
                    if mag_q is not None and plots[-1][0].is_alive():
                        mag_q.put((d.t, d.mag_x, d.mag_y, d.mag_z))
                    if display_every and n_ok % display_every == 0:
                        if block_mode:
                            # カーソルを BLOCK_LINES 行上げてブロックを上書き（ESC[K で各行末まで消去）
                            body = format_block(d, n_ok, n_err).replace("\n", "\x1b[K\n")
                            sys.stdout.write(f"\x1b[{BLOCK_LINES}A\r{body}\x1b[K\n")
                            sys.stdout.flush()
                        else:
                            print(format_line(d))
                except BNO055Error as e:
                    n_err += 1
                    print(f"読み出しエラー: {e}", file=sys.stderr)

                now = time.perf_counter()
                if args.duration is not None and now - t_start >= args.duration:
                    break
                # 定周期化: 次の予定時刻まで待つ。遅延が溜まった場合は追いつくためスキップ
                if next_t - now > 0:
                    time.sleep(next_t - now)
                elif now - next_t > period:
                    next_t = now
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.perf_counter() - t_start
            if csv_file:
                csv_file.close()
            print(f"\n終了: {elapsed:.2f} s, 取得 {n_ok} サンプル ({n_ok / elapsed if elapsed else 0:.1f} Hz), エラー {n_err} 回")
            alive = [(p, q) for p, q in plots if p.is_alive()]
            if alive:
                for p, q in alive:
                    q.put(None)   # 記録終了を通知（グラフは残る）
                print("グラフウィンドウを閉じると終了します")
                for p, _ in alive:
                    p.join()
    return 0


if __name__ == "__main__":
    sys.exit(main())
