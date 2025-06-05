# SwitchRoboSimu2 シミュレーション環境

本プロジェクトは、Choreonoid上で動作する仮想ロボットを用いたスイッチ操作シミュレーションです。複数のコントローラと仮想ジョイスティックを連携させ、環境内のスイッチやボタンを制御することを目的としています。

---

## 🔧 使用ファイルと構成

### シミュレーションファイル
- `SwitchRoboSimu2.cnoid`：ロボットモデルと環境を定義したChoreonoidプロジェクトファイル

### ⚙️ コントローラ一覧

| ファイル名 | 役割 |
|------------|------|
| `AutoBoxController3.cpp` | 移動制御、仮想ジョイスティック入力の処理 |
| `CylinderLEDLightController2.cpp` | スイッチ（Cylinder型）の回転制御 |
| `CameraSample_bac.cpp` | カメラによる画像取得や視界シミュレーション |

---

## 🕹️ 仮想ジョイスティック操作対応

| ボタン | 動作 |
|--------|------|
| Yボタン | 左端のスイッチを回しに行く |
| Aボタン | 中央のボタンを回す |
| Bボタン | 右端のスイッチを回しに行く |

---

## 🔨 ビルド方法

以下の手順で各コントローラをビルドしてください：

```bash
cd path/to/controllers/
mkdir build
cd build
cmake ..
make
```

---

## 📄 ライセンス

以下がライセンス情報になります:

BSD 2-Clause License  
Copyright (c) 2025, アクアクルー株式会社  
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,  
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,  
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,  
   this list of conditions and the following disclaimer in the documentation  
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED  
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,  
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;  
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND  
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS  
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
