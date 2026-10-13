"""
リアルタイムグラフ（imu_read.py --plot / --plot-mag 用）。

--plot:     3 段のサブプロット（上: 加速度 x/y/z [m/s^2]、中: 角速度 x/y/z [deg/s]、下: オイラー角 roll/pitch/heading [deg]）。
            オイラー角は回転軸が x, y, z の順になる roll, pitch, heading の順に並べ、線の色を x/y/z と揃える。
            layout="raw"（AMG などの非フュージョンモード用）では下段がオイラー角の代わりに地磁気 x/y/z [uT] になる。
--plot-mag: 別ウィンドウに地磁気を 2 段で表示（上: x/y/z 成分 [uT]、下: 大きさ |B| [uT]）。
            センサを回転させても |B| は変わらないが、磁石や鉄・電子機器が近づくと |B| が変わるので、外乱の検出に使える。

どちらも直近 window_s 秒ぶんをスクロール表示する。横軸（時間）は全段で共通、縦軸は段ごとに自動スケール。

サンプリングの定周期を乱さないよう、グラフは別プロセスで描画する。
親プロセスは multiprocessing.Queue に (t, 値...) のタプルを入れるだけで、子プロセス側が一定間隔（PLOT_INTERVAL_MS）に
まとめて取り出して描画する。Queue に None を入れると「記録終了」とみなし、ウィンドウはそのまま残して更新だけ止める。

単体テスト用:
    python live_plot.py        # 疑似データを流して両方のグラフを表示
"""

from __future__ import annotations

import math
import signal
from collections import deque
from multiprocessing import Queue

PLOT_INTERVAL_MS = 100   # 描画更新間隔 [ms]（10 fps）
MAX_POINTS_DRAWN = 1500  # 1 本の線に描く最大点数（超えたら間引いて描画負荷を抑える）

# 色（dataviz 参照パレット: 系列 1 = 青, 2 = 橙, 3 = 青緑。文字・軸は中立色）
# 同じ色が全パネルで同じ成分（1 番目 / 2 番目 / 3 番目）を表すようにする
COLORS = ("#2a78d6", "#eb6834", "#1baf7a")
COLOR_TEXT = "#0b0b0b"
COLOR_TEXT_2 = "#52514e"
COLOR_GRID = "#e4e3df"
COLOR_SURFACE = "#fcfcfb"

# パネル定義: (タイトル, 単位, 系列名のタプル, 縦軸の最小レンジ)
PANEL_ACC = ("Acceleration", "[m/s$^2$]", ("x", "y", "z"), 2.0)
PANEL_GYR = ("Angular velocity", "[deg/s]", ("x", "y", "z"), 20.0)
PANEL_EUL = ("Euler angles", "[deg]", ("roll", "pitch", "heading"), 20.0)   # 回転軸の順 (x, y, z) に合わせる
PANEL_MAG = ("Magnetic field", "[uT]", ("x", "y", "z"), 20.0)
PANEL_MAG_NORM = ("|B|", "[uT]", ("|B|",), 10.0)
LAYOUTS = {
    "fusion": (PANEL_ACC, PANEL_GYR, PANEL_EUL),   # フュージョンモード: 下段はオイラー角
    "raw": (PANEL_ACC, PANEL_GYR, PANEL_MAG),      # 非フュージョンモード（AMG 等）: 下段は地磁気
    "mag": (PANEL_MAG, PANEL_MAG_NORM),            # --plot-mag 用
}


def _autoscale(ax, series: list[deque], min_span: float) -> None:
    """表示中データの範囲に上下 15% の余白を付けて y 軸を合わせる。範囲が min_span 未満なら中心を保って広げる。"""
    lo = min(min(s) for s in series)
    hi = max(max(s) for s in series)
    span = max(hi - lo, min_span)
    mid = (lo + hi) / 2
    ax.set_ylim(mid - span * 0.65, mid + span * 0.65)


def _run(q: Queue, panels: tuple, window_s: float, title: str, figsize: tuple, auto_close: bool) -> None:
    """共通の描画ループ。Queue の各要素は (t, 系列値...) で、系列値はパネル順・系列順に並ぶ。"""
    # Ctrl+C は親プロセスが処理する（子はコンソールの Ctrl+C を無視し、親からの None で終了状態になる）
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    import matplotlib
    import matplotlib.pyplot as plt

    matplotlib.rcParams.update({
        "figure.facecolor": COLOR_SURFACE, "axes.facecolor": COLOR_SURFACE,
        "text.color": COLOR_TEXT, "axes.labelcolor": COLOR_TEXT_2,
        "xtick.color": COLOR_TEXT_2, "ytick.color": COLOR_TEXT_2,
        "axes.edgecolor": COLOR_GRID, "grid.color": COLOR_GRID,
        "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "font.size": 10,
    })

    ts: deque[float] = deque()
    data: list[list[deque]] = [[deque() for _ in names] for (_, _, names, _) in panels]   # data[panel][series]

    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=figsize, squeeze=False)
    axes = axes[:, 0]
    fig.canvas.manager.set_window_title(f"{title} live plot")
    fig.suptitle(f"{title}   " + " / ".join(p[0] for p in panels), color=COLOR_TEXT)

    lines: list[list] = []
    labels: list = []
    lbl_kw = dict(ha="right", va="top", color=COLOR_TEXT, fontsize=11, family="monospace",
                  bbox=dict(boxstyle="round,pad=0.25", facecolor=COLOR_SURFACE, edgecolor="none", alpha=0.85))
    for ax, (name, unit, names, _) in zip(axes, panels):
        ls = [ax.plot([], [], color=COLORS[i], linewidth=1.4, label=n)[0] for i, n in enumerate(names)]
        lines.append(ls)
        ax.set_ylabel(f"{name} {unit}")
        ax.grid(True, linewidth=0.6)
        ax.set_xlim(0, window_s)
        if len(names) >= 2:   # 系列が 1 本のパネルは凡例不要（タイトルが系列名）
            ax.legend(loc="upper left", ncol=len(names), fontsize=9)
        labels.append(ax.text(0.99, 0.95, "", transform=ax.transAxes, **lbl_kw))   # 最新値の直接ラベル
    axes[-1].set_xlabel("Time [s]")
    status = fig.text(0.01, 0.01, "recording...", color=COLOR_TEXT_2, fontsize=9)
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))

    state = {"done": False}

    def update() -> None:
        if state["done"]:
            return
        got = False
        while True:
            try:
                item = q.get_nowait()
            except Exception:
                break
            if item is None:
                state["done"] = True
                break
            ts.append(item[0])
            k = 1
            for pd in data:
                for s in pd:
                    s.append(item[k])
                    k += 1
            got = True
        if got:
            # 表示範囲外の古いデータを捨てる
            t_last = ts[-1]
            while ts and ts[0] < t_last - window_s:
                ts.popleft()
                for pd in data:
                    for s in pd:
                        s.popleft()
            x0 = max(0.0, t_last - window_s)
            stride = max(1, len(ts) // MAX_POINTS_DRAWN)
            xs = list(ts)[::stride]
            for pi, (_, _, names, min_span) in enumerate(panels):
                for si in range(len(names)):
                    lines[pi][si].set_data(xs, list(data[pi][si])[::stride])
                _autoscale(axes[pi], data[pi], min_span)
                labels[pi].set_text("  ".join(f"{n}={data[pi][si][-1]:8.2f}" for si, n in enumerate(names)))
            axes[-1].set_xlim(x0, x0 + window_s)
            fig.canvas.draw_idle()
        if state["done"]:
            status.set_text("finished (close this window to exit)")
            fig.canvas.draw_idle()
            timer.stop()
            if auto_close:
                plt.close(fig)

    timer = fig.canvas.new_timer(interval=PLOT_INTERVAL_MS)
    timer.add_callback(update)
    timer.start()
    plt.show()


def run_plot(q: Queue, window_s: float = 20.0, title: str = "BNO055", layout: str = "fusion",
             auto_close: bool = False) -> None:
    """--plot 用の子プロセスエントリ。Queue の要素は (t, ax, ay, az, gx, gy, gz, 下段の 3 値)。"""
    _run(q, LAYOUTS[layout], window_s, title, (9, 8), auto_close)


def run_mag_plot(q: Queue, window_s: float = 20.0, title: str = "BNO055", auto_close: bool = False) -> None:
    """--plot-mag 用の子プロセスエントリ。Queue の要素は (t, mx, my, mz)。|B| はここで計算する。"""
    class NormQueue:
        """(t, mx, my, mz) を (t, mx, my, mz, |B|) に変換して渡すラッパ"""
        def get_nowait(self):
            item = q.get_nowait()
            if item is None:
                return None
            t, mx, my, mz = item
            return (t, mx, my, mz, math.sqrt(mx * mx + my * my + mz * mz))
    _run(NormQueue(), LAYOUTS["mag"], window_s, title, (9, 5.5), auto_close)


if __name__ == "__main__":
    # 疑似データで単体テスト: ゆっくり回転しながら揺れている IMU と、途中で磁石が近づく地磁気
    import threading
    import time
    from multiprocessing import Process, Queue as MPQueue

    q: Queue = MPQueue()
    qm: Queue = MPQueue()

    def feeder() -> None:
        t0 = time.perf_counter()
        while (t := time.perf_counter() - t0) < 12:
            h = (t * 30) % 360
            r = 15 * math.sin(t * 1.5)
            p = 10 * math.sin(t * 0.7)
            ax, ay, az = 2 * math.sin(t * 3), 1.5 * math.cos(t * 2), 9.8 + 0.5 * math.sin(t * 5)
            gx, gy, gz = 22 * math.cos(t * 1.5), 7 * math.cos(t * 0.7), 30.0
            q.put((t, ax, ay, az, gx, gy, gz, r, p, h))
            th = math.radians(h)
            dist = 40 if 5 < t < 8 else 0   # 5〜8 秒に磁石が近づく
            qm.put((t, 30 * math.cos(th) + dist, 30 * math.sin(th), -35 + 0.5 * dist))
            time.sleep(0.01)
        q.put(None)
        qm.put(None)

    threading.Thread(target=feeder, daemon=True).start()
    pm = Process(target=run_mag_plot, args=(qm, 10, "demo"), daemon=True)
    pm.start()
    run_plot(q, window_s=10, title="demo")
    pm.join()
