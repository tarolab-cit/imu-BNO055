# WSL (Ubuntu 24.04) で動かす

WSL2 は Windows の COM ポートを直接使えないため、[usbipd-win](https://github.com/dorssel/usbipd-win) で USB シリアル変換器を WSL に接続
プログラムの変更は不要で、`--port /dev/ttyUSB0` を付けて実行する

## 1. usbipd-win のインストール（Windows、初回のみ）

管理者 PowerShell で:

```powershell
winget install dorssel.usbipd-win
```

## 2. USB シリアル変換器を WSL に接続（Windows）

USB シリアル変換器を挿して BUSID を確認し、初回だけ bind（管理者）、以降は attach する。

```powershell
usbipd list                          # 例: 1-3  067b:23a3  Prolific PL2303GC USB Serial COM Port (COM7)
usbipd bind --busid 1-3              # 初回のみ、管理者
usbipd attach --wsl --busid 1-3      # 挿し直し・再起動のたびに実行（--auto-attach で自動再接続）
```

attach 中は Windows 側から COM ポートが使えない。戻すときは `usbipd detach --busid 1-3`

## 3. WSL 側の設定（Ubuntu、初回のみ）

```bash
ls /dev/ttyUSB*                      # /dev/ttyUSB0 が見えれば OK
sudo usermod -aG dialout $USER       # シリアルポートの権限追加、反映には端末を開き直す
sudo apt install python3-serial python3-matplotlib python3-tk
```

pip を使う場合は venv を作る:

```bash
sudo apt install python3-venv python3-tk
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## 4. 実行

```bash
python3 imu_read.py --port /dev/ttyUSB0 --plot --csv
python3 imu_calibrate.py --port /dev/ttyUSB0
```
