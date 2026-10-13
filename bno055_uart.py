"""
BNO055 (Bosch 9軸センサフュージョンIMU) を UART モードで制御するドライバ。

対象ボード: 秋月電子 AE-BNO055-BO（J2短絡・J3/J4/J5開放で UART モード）
UART 仕様: 115200 bps, 8N1

プロトコル（BNO055 データシート 4.7 UART Protocol）
  書き込み要求: 0xAA 0x00 <reg> <len> <data...>   応答: 0xEE <status>
  読み出し要求: 0xAA 0x01 <reg> <len>             応答: 0xBB <len> <data...>  または 0xEE <status>
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, fields

import serial

# ---------------------------------------------------------------------------
# レジスタ（Page 0）
# ---------------------------------------------------------------------------
REG_CHIP_ID = 0x00
REG_PAGE_ID = 0x07
REG_ACC_DATA_X_LSB = 0x08
REG_MAG_DATA_X_LSB = 0x0E
REG_GYR_DATA_X_LSB = 0x14
REG_EUL_HEADING_LSB = 0x1A
REG_QUA_DATA_W_LSB = 0x20
REG_LIA_DATA_X_LSB = 0x28
REG_GRV_DATA_X_LSB = 0x2E
REG_TEMP = 0x34
REG_CALIB_STAT = 0x35
REG_ST_RESULT = 0x36
REG_SYS_STATUS = 0x39
REG_SYS_ERR = 0x3A
REG_UNIT_SEL = 0x3B
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_ACC_OFFSET_X_LSB = 0x55   # 校正オフセット 0x55..0x6A (22 バイト)
CALIB_OFFSET_LEN = 22

CHIP_ID = 0xA0

# 動作モード（OPR_MODE）
MODE_CONFIG = 0x00
MODE_ACCONLY = 0x01
MODE_MAGONLY = 0x02
MODE_GYRONLY = 0x03
MODE_ACCMAG = 0x04
MODE_ACCGYRO = 0x05
MODE_MAGGYRO = 0x06
MODE_AMG = 0x07       # 9軸生データのみ（フュージョンなし）
MODE_IMU = 0x08       # 加速度+ジャイロのフュージョン（相対姿勢）
MODE_COMPASS = 0x09
MODE_M4G = 0x0A
MODE_NDOF_FMC_OFF = 0x0B
MODE_NDOF = 0x0C      # 9軸フュージョン（絶対姿勢）

MODE_NAMES = {
    "config": MODE_CONFIG,
    "acconly": MODE_ACCONLY,
    "magonly": MODE_MAGONLY,
    "gyronly": MODE_GYRONLY,
    "accmag": MODE_ACCMAG,
    "accgyro": MODE_ACCGYRO,
    "maggyro": MODE_MAGGYRO,
    "amg": MODE_AMG,
    "imu": MODE_IMU,
    "compass": MODE_COMPASS,
    "m4g": MODE_M4G,
    "ndof_fmc_off": MODE_NDOF_FMC_OFF,
    "ndof": MODE_NDOF,
}
FUSION_MODES = {MODE_IMU, MODE_COMPASS, MODE_M4G, MODE_NDOF_FMC_OFF, MODE_NDOF}


def is_fusion_mode(mode: int) -> bool:
    """フュージョンモード（オイラー角・クォータニオンが出るモード）か"""
    return mode in FUSION_MODES


# ---------------------------------------------------------------------------
# センサ設定レジスタ（Page 1）。フュージョンモードでは加速度レンジ以外は自動設定され、変更しても無視される。
# ---------------------------------------------------------------------------
REG_ACC_CONFIG = 0x08     # bit1:0 レンジ, bit4:2 帯域幅, bit7:5 電源モード
REG_MAG_CONFIG = 0x09     # bit2:0 出力レート, bit4:3 動作モード, bit6:5 電源モード
REG_GYR_CONFIG_0 = 0x0A   # bit2:0 レンジ, bit5:3 帯域幅
REG_GYR_CONFIG_1 = 0x0B   # bit2:0 電源モード

ACC_RANGE_TABLE = {2: 0, 4: 1, 8: 2, 16: 3}                       # [g]
ACC_BW_TABLE = {7.81: 0, 15.63: 1, 31.25: 2, 62.5: 3, 125: 4, 250: 5, 500: 6, 1000: 7}   # [Hz]
GYR_RANGE_TABLE = {2000: 0, 1000: 1, 500: 2, 250: 3, 125: 4}       # [deg/s]
GYR_BW_TABLE = {523: 0, 230: 1, 116: 2, 47: 3, 23: 4, 12: 5, 64: 6, 32: 7}                # [Hz]
MAG_RATE_TABLE = {2: 0, 6: 1, 8: 2, 10: 3, 15: 4, 20: 5, 25: 6, 30: 7}                    # [Hz]

# UART 応答ステータス
UART_STATUS = {
    0x01: "WRITE_SUCCESS",
    0x02: "READ_FAIL",
    0x03: "WRITE_FAIL",
    0x04: "REGMAP_INVALID_ADDRESS",
    0x05: "REGMAP_WRITE_DISABLED",
    0x06: "WRONG_START_BYTE",
    0x07: "BUS_OVER_RUN_ERROR",
    0x08: "MAX_LENGTH_ERROR",
    0x09: "MIN_LENGTH_ERROR",
    0x0A: "RECEIVE_CHARACTER_TIMEOUT",
}

# 単位換算（UNIT_SEL デフォルト: m/s^2, dps, deg, °C, Android 向き）
ACC_LSB_PER_MS2 = 100.0
MAG_LSB_PER_UT = 16.0
GYR_LSB_PER_DPS = 16.0
EUL_LSB_PER_DEG = 16.0
QUA_LSB_PER_UNIT = 16384.0  # 2^14


class BNO055Error(Exception):
    """BNO055 通信エラー"""


@dataclass
class IMUData:
    """BNO055 の全出力を 1 回分まとめたデータ"""
    t: float                      # 取得時刻 [s]（time.perf_counter 基準、開始からの相対時間）
    acc_x: float; acc_y: float; acc_z: float        # 加速度 [m/s^2]
    mag_x: float; mag_y: float; mag_z: float        # 地磁気 [uT]
    gyr_x: float; gyr_y: float; gyr_z: float        # 角速度 [deg/s]
    eul_heading: float; eul_roll: float; eul_pitch: float  # オイラー角 [deg]
    qua_w: float; qua_x: float; qua_y: float; qua_z: float  # クォータニオン [-]
    lia_x: float; lia_y: float; lia_z: float        # 線形加速度（重力除去） [m/s^2]
    grv_x: float; grv_y: float; grv_z: float        # 重力ベクトル [m/s^2]
    temp: int                                       # 温度 [°C]
    calib_sys: int; calib_gyr: int; calib_acc: int; calib_mag: int  # キャリブレーション状態 0..3

    @classmethod
    def field_names(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    def as_list(self) -> list:
        return [getattr(self, n) for n in self.field_names()]


class BNO055UART:
    """BNO055 UART ドライバ"""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2, retries: int = 5):
        self.ser = serial.Serial(port, baudrate, timeout=timeout, write_timeout=timeout)
        self.retries = retries
        self._t0 = time.perf_counter()
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    # ----- コンテキストマネージャ -----
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ----- 低レベル通信 -----
    def _read_exact(self, n: int) -> bytes:
        buf = self.ser.read(n)
        if len(buf) != n:
            raise BNO055Error(f"タイムアウト: {n} バイト期待, {len(buf)} バイト受信")
        return buf

    def read_regs(self, reg: int, length: int) -> bytes:
        """レジスタを length バイト読み出す（最大 128 バイト）。バスオーバーラン等はリトライ。"""
        if not 1 <= length <= 128:
            raise ValueError("length は 1..128")
        last_err = None
        for _ in range(self.retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(bytes([0xAA, 0x01, reg & 0xFF, length]))
                hdr = self._read_exact(2)
                if hdr[0] == 0xBB:
                    n = hdr[1]
                    data = self._read_exact(n)
                    if n != length:
                        raise BNO055Error(f"長さ不一致: 期待 {length}, 受信 {n}")
                    return data
                if hdr[0] == 0xEE:
                    status = UART_STATUS.get(hdr[1], f"0x{hdr[1]:02X}")
                    last_err = BNO055Error(f"読み出しエラー reg=0x{reg:02X}: {status}")
                else:
                    last_err = BNO055Error(f"不正な応答ヘッダ: {hdr.hex(' ')}")
            except BNO055Error as e:
                last_err = e
            time.sleep(0.002)
        raise last_err  # type: ignore[misc]

    def write_regs(self, reg: int, data: bytes, expect_reply: bool = True) -> None:
        """レジスタに書き込む。expect_reply=False はリセット等で応答が返らない場合用。"""
        last_err = None
        for attempt in range(self.retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(bytes([0xAA, 0x00, reg & 0xFF, len(data)]) + bytes(data))
                if not expect_reply:
                    return
                hdr = self._read_exact(2)
                if hdr[0] == 0xEE and hdr[1] == 0x01:
                    return
                status = UART_STATUS.get(hdr[1], f"0x{hdr[1]:02X}") if hdr[0] == 0xEE else hdr.hex(" ")
                last_err = BNO055Error(f"書き込みエラー reg=0x{reg:02X}: {status}")
            except BNO055Error as e:
                last_err = e
            # BUS_OVER_RUN_ERROR は直前の処理（モード切替・ページ切替）が終わる前に書いたときに出るので、待ってから再試行
            time.sleep(0.01 * (attempt + 1))
        raise last_err  # type: ignore[misc]

    def read_reg(self, reg: int) -> int:
        return self.read_regs(reg, 1)[0]

    def write_reg(self, reg: int, value: int) -> None:
        self.write_regs(reg, bytes([value & 0xFF]))

    # ----- 初期化・設定 -----
    def begin(self, mode: int = MODE_NDOF, reset: bool = False) -> None:
        """
        接続確認と動作モード設定。
        reset=True の場合はソフトリセットを行う（キャリブレーション状態は失われる）。
        """
        chip_id = self.read_reg(REG_CHIP_ID)
        if chip_id != CHIP_ID:
            raise BNO055Error(f"CHIP_ID 不一致: 0x{chip_id:02X} (期待 0x{CHIP_ID:02X})")

        self.set_mode(MODE_CONFIG)
        if reset:
            self.write_reg(REG_SYS_TRIGGER, 0x20)   # RST_SYS
            time.sleep(0.7)
            # リセット後、CHIP_ID が読めるまで待つ
            for _ in range(20):
                try:
                    if self.read_reg(REG_CHIP_ID) == CHIP_ID:
                        break
                except BNO055Error:
                    pass
                time.sleep(0.05)
            else:
                raise BNO055Error("リセット後に応答がありません")
            time.sleep(0.05)

        self.write_reg(REG_PAGE_ID, 0x00)
        self.write_reg(REG_PWR_MODE, 0x00)      # Normal power mode
        self.write_reg(REG_UNIT_SEL, 0x80)      # m/s^2, dps, deg, °C, Android 向き（=デフォルト）
        self.write_reg(REG_SYS_TRIGGER, 0x00)   # 内部発振（外部水晶を使う場合は 0x80）
        self.set_mode(mode)

    def set_mode(self, mode: int) -> None:
        """動作モードを切り替える。動作モード間の直接切替は失敗することがあるため、必ず CONFIG を経由する。"""
        mode &= 0x0F
        if mode != MODE_CONFIG and self.get_mode() != MODE_CONFIG:
            self.write_reg(REG_OPR_MODE, MODE_CONFIG)
            time.sleep(0.03)
        self.write_reg(REG_OPR_MODE, mode)
        # データシート: CONFIG→他 7ms, 他→CONFIG 19ms
        time.sleep(0.03)

    def get_mode(self) -> int:
        return self.read_reg(REG_OPR_MODE) & 0x0F

    def get_system_status(self) -> tuple[int, int, int]:
        """(SYS_STATUS, ST_RESULT, SYS_ERR) を返す"""
        return (self.read_reg(REG_SYS_STATUS),
                self.read_reg(REG_ST_RESULT),
                self.read_reg(REG_SYS_ERR))

    # ----- センサ設定（Page 1） -----
    def _with_page1(self, func):
        """CONFIG モード・Page 1 で func(self) を実行し、Page 0・元のモードに戻す"""
        mode = self.get_mode()
        self.set_mode(MODE_CONFIG)
        try:
            self.write_reg(REG_PAGE_ID, 0x01)
            try:
                return func(self)
            finally:
                self.write_reg(REG_PAGE_ID, 0x00)
        finally:
            self.set_mode(mode)

    def get_sensor_config(self) -> dict:
        """加速度・ジャイロ・地磁気の設定を読んで dict で返す（acc_range [g], acc_bw [Hz], gyr_range [deg/s], gyr_bw [Hz], mag_rate [Hz]）"""
        def read(imu):
            acc, mag, gyr0, _ = imu.read_regs(REG_ACC_CONFIG, 4)
            return acc, mag, gyr0
        acc, mag, gyr0 = self._with_page1(read)
        inv = lambda table, code: next(k for k, v in table.items() if v == code)
        return {
            "acc_range": inv(ACC_RANGE_TABLE, acc & 0x03),
            "acc_bw": inv(ACC_BW_TABLE, (acc >> 2) & 0x07),
            "gyr_range": inv(GYR_RANGE_TABLE, gyr0 & 0x07),
            "gyr_bw": inv(GYR_BW_TABLE, (gyr0 >> 3) & 0x07),
            "mag_rate": inv(MAG_RATE_TABLE, mag & 0x07),
        }

    def set_sensor_config(self, acc_range: int | None = None, acc_bw: float | None = None,
                          gyr_range: int | None = None, gyr_bw: int | None = None,
                          mag_rate: int | None = None) -> None:
        """
        加速度レンジ [g]、加速度帯域幅 [Hz]、ジャイロレンジ [deg/s]、ジャイロ帯域幅 [Hz]、地磁気出力レート [Hz] を設定する。
        None の項目は変更しない。帯域幅はセンサ内部のローパスフィルタのカットオフに相当する。
        注意: フュージョンモード（NDOF 等）では加速度レンジ以外は無視され、モード切替時に既定値へ戻される。
        """
        for v, table, name in ((acc_range, ACC_RANGE_TABLE, "acc_range"), (acc_bw, ACC_BW_TABLE, "acc_bw"),
                               (gyr_range, GYR_RANGE_TABLE, "gyr_range"), (gyr_bw, GYR_BW_TABLE, "gyr_bw"),
                               (mag_rate, MAG_RATE_TABLE, "mag_rate")):
            if v is not None and v not in table:
                raise ValueError(f"{name} は {sorted(table)} のいずれか")

        def write(imu):
            acc, mag, gyr0, _ = imu.read_regs(REG_ACC_CONFIG, 4)
            if acc_range is not None:
                acc = (acc & ~0x03) | ACC_RANGE_TABLE[acc_range]
            if acc_bw is not None:
                acc = (acc & ~(0x07 << 2)) | (ACC_BW_TABLE[acc_bw] << 2)
            if gyr_range is not None:
                gyr0 = (gyr0 & ~0x07) | GYR_RANGE_TABLE[gyr_range]
            if gyr_bw is not None:
                gyr0 = (gyr0 & ~(0x07 << 3)) | (GYR_BW_TABLE[gyr_bw] << 3)
            if mag_rate is not None:
                mag = (mag & ~0x07) | MAG_RATE_TABLE[mag_rate]
            imu.write_reg(REG_ACC_CONFIG, acc)
            imu.write_reg(REG_MAG_CONFIG, mag)
            imu.write_reg(REG_GYR_CONFIG_0, gyr0)
        self._with_page1(write)

    def get_calibration(self) -> tuple[int, int, int, int]:
        """(sys, gyr, acc, mag) のキャリブレーション状態（各 0..3、3 が完全）"""
        c = self.read_reg(REG_CALIB_STAT)
        return (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3

    def read_calibration_offsets(self) -> bytes:
        """
        校正オフセット 22 バイト（acc 6, mag 6, gyr 6, acc_radius 2, mag_radius 2）を読み出す。
        データシートに従い CONFIG モードに切り替えて読み、元のモードに戻す。
        """
        mode = self.get_mode()
        self.set_mode(MODE_CONFIG)
        try:
            return self.read_regs(REG_ACC_OFFSET_X_LSB, CALIB_OFFSET_LEN)
        finally:
            self.set_mode(mode)

    def write_calibration_offsets(self, offsets: bytes) -> None:
        """保存しておいた校正オフセット 22 バイトを書き戻す（CONFIG モードで書き込み、元のモードに戻す）。"""
        if len(offsets) != CALIB_OFFSET_LEN:
            raise ValueError(f"offsets は {CALIB_OFFSET_LEN} バイト")
        mode = self.get_mode()
        self.set_mode(MODE_CONFIG)
        try:
            # BNO055 の UART は複数バイト一括書き込みで BUS_OVER_RUN_ERROR を返すため 1 バイトずつ書く
            for i, b in enumerate(offsets):
                self.write_reg(REG_ACC_OFFSET_X_LSB + i, b)
        finally:
            self.set_mode(mode)

    @staticmethod
    def decode_offsets(offsets: bytes) -> dict:
        """22 バイトの校正オフセットを人が読める dict にする"""
        v = struct.unpack("<3h3h3hhh", offsets)
        return {
            "acc_offset": list(v[0:3]), "mag_offset": list(v[3:6]), "gyr_offset": list(v[6:9]),
            "acc_radius": v[9], "mag_radius": v[10],
        }

    # ----- データ取得 -----
    def read_all(self) -> IMUData:
        """
        0x08 (ACC_DATA_X_LSB) から 0x35 (CALIB_STAT) までの 46 バイトを一括読み出しし、
        物理量に換算して IMUData を返す。1 回の UART トランザクションで済むため高速。
        """
        t = time.perf_counter() - self._t0
        raw = self.read_regs(REG_ACC_DATA_X_LSB, 46)
        # acc(3h) mag(3h) gyr(3h) eul(3h) qua(4h) lia(3h) grv(3h) temp(b) calib(B)
        v = struct.unpack("<3h3h3h3h4h3h3hbB", raw)
        acc = [x / ACC_LSB_PER_MS2 for x in v[0:3]]
        mag = [x / MAG_LSB_PER_UT for x in v[3:6]]
        gyr = [x / GYR_LSB_PER_DPS for x in v[6:9]]
        eul = [x / EUL_LSB_PER_DEG for x in v[9:12]]
        qua = [x / QUA_LSB_PER_UNIT for x in v[12:16]]
        lia = [x / ACC_LSB_PER_MS2 for x in v[16:19]]
        grv = [x / ACC_LSB_PER_MS2 for x in v[19:22]]
        temp = v[22]
        c = v[23]
        return IMUData(
            t, *acc, *mag, *gyr, *eul, *qua, *lia, *grv, temp,
            (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3,
        )

    def read_vector(self, reg: int, n: int = 3, scale: float = 1.0) -> list[float]:
        """任意のベクトルレジスタ（int16 × n）を読み出して換算"""
        raw = self.read_regs(reg, 2 * n)
        return [x / scale for x in struct.unpack(f"<{n}h", raw)]
