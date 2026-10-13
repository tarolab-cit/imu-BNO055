"""
BNO055 のキャリブレーション補助プログラム。

BNO055 はフュージョンモード（NDOF など）で動作中、センサを動かすだけで内部で自動的に
校正が進みます。専用の校正コマンドはありません。このプログラムは校正状態を画面に
表示しながら手順を案内し、全項目が 3（完了）になったら校正オフセットを JSON に保存します。
保存した校正値は次回以降 `imu_read.py --calib bno055_calib.json` で書き戻せます。

使い方:
    python imu_calibrate.py                 # COM5, 保存先 bno055_calib.json
    python imu_calibrate.py --port COM5 --out mycal.json
    python imu_calibrate.py --save-now      # 3 になるのを待たず現在値を保存して終了

手順:
    1. ジャイロ  : センサを机の上に置いて数秒動かさない
    2. 加速度    : +X/-X/+Y/-Y/+Z/-Z の各面を上にして、それぞれ 2〜3 秒静置（6 面）
                   面を変えるときはゆっくり動かす
    3. 地磁気    : センサを空中で「8 の字」にゆっくり大きく動かす。PC や金属から離す
    4. システム  : 上記 3 つが 3 になり、少し静置すると 3 になる
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime

from bno055_uart import BNO055UART, BNO055Error, MODE_NDOF

BAR = {0: "[   ]", 1: "[*  ]", 2: "[** ]", 3: "[***]"}


def main() -> int:
    ap = argparse.ArgumentParser(description="BNO055 キャリブレーション補助")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="bno055_calib.json", help="校正値の保存先 (default: bno055_calib.json)")
    ap.add_argument("--save-now", action="store_true", help="校正完了を待たず現在の校正値を保存して終了")
    args = ap.parse_args()

    with BNO055UART(args.port, args.baud) as imu:
        imu.begin(MODE_NDOF)

        if not args.save_now:
            print(__doc__.split("手順:")[1])
            print("校正状態（3 が完了）。全項目 3 になると自動保存します。Ctrl+C で中断。\n")
            try:
                while True:
                    d = imu.read_all()
                    sys_c, gyr, acc, mag = d.calib_sys, d.calib_gyr, d.calib_acc, d.calib_mag
                    line = (f"\rGYR {BAR[gyr]}  ACC {BAR[acc]}  MAG {BAR[mag]}  SYS {BAR[sys_c]}   "
                            f"heading={d.eul_heading:6.1f} roll={d.eul_roll:6.1f} pitch={d.eul_pitch:6.1f}   ")
                    print(line, end="", flush=True)
                    if sys_c == gyr == acc == mag == 3:
                        print("\n\n全項目の校正が完了しました。")
                        break
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n中断しました。校正値は保存しません。")
                return 1

        offsets = imu.read_calibration_offsets()
        calib = imu.get_calibration()
        data = {
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "calib_status": {"sys": calib[0], "gyr": calib[1], "acc": calib[2], "mag": calib[3]},
            "offsets_hex": offsets.hex(),
            **BNO055UART.decode_offsets(offsets),
        }
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"校正値を保存しました: {args.out}")
        print(json.dumps(data, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BNO055Error as e:
        print(f"BNO055 エラー: {e}", file=sys.stderr)
        sys.exit(1)
