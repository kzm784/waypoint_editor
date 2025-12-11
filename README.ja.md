[English](README.md) | [日本語](README.ja.md)

[![ROS 2 Humble build](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml/badge.svg?branch=main&label=ROS%202%20Humble%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml)
[![ROS 2 Jazzy build](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml/badge.svg?branch=main&label=ROS%202%20Jazzy%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml)

# Waypoint Editor

![demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/waypoint_editor_demo.gif)


## 目次
- [概要](#概要)
- [開発環境](#開発環境)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)
- [ライセンス](#ライセンス)


## 概要
このパッケージは、ナビゲーションで使用するウェイポイント（Waypoint）を、2D/3D 地図を見ながら直感的に編集・保存できるツールです。  
編集したウェイポイントは **CSV形式** や Nav2 互換の **YAML形式** で保存でき、自己位置トピックから自動追加も可能です。


## 開発環境
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill


## インストール方法
以下のコマンドをターミナルで実行してください：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kzm784/waypoint_editor.git
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## 使用方法
### 1. Waypoint Editor の起動  
以下のコマンドでツールを起動します：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch waypoint_editor waypoint_editor.launch.py
```

### 2. Map
#### 地図の読み込み (2D / 3D)  
- Nav2 の `nav2_map_server` を利用して `.yaml` 形式の2Dマップを読み込みます。  
- `.pcd` 形式の3Dマップを読み込むことも可能です。  
- RViz2 パネルの "**Load 2D Map**" ボタンで `.yaml` ファイルを、"**Load 3D Map**" ボタンで `.pcd` ファイルを選択してください。

![load_map_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/load_map_demo.gif)


### 3. Waypoints
#### ウェイポイントの追加  
- RViz2 画面上部のツールバーから "**Add Waypoint**" を選択します。  
- 地図上でドラッグ＆ドロップすることで、位置と向きを指定して新しいウェイポイントを追加できます。  
- 追加されたウェイポイントは：
  - **ドラッグで移動・回転**が可能
  - **右クリックでメニュー表示**から削除や編集が可能

![interact_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/interact_waypoints_demo.gif)

#### ウェイポイントの保存  
- RViz2 画面右下のパネルで "**Save WPs**" を選択し、ダイアログから **CSV** または **YAML** を選べます。  
- Nav2 で利用したい場合は **YAML** を選択してください。

![save_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/save_waypoints_demo.gif)

#### ウェイポイントの読み込み  
- "**Load WPs**" ボタンから、保存しておいた `.csv` または `.yaml` ファイルを読み込み、ウェイポイントの再編集が可能です。

![load_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/load_waypoints_demo.gif)

#### 操作の取り消し / やり直し / 全消去  
- パネルの "**Undo**" / "**Redo**" ボタンで、移動・回転・削除・ID変更・コマンド編集といった操作を戻したり進めたりできます。  
- "**Clear All**" で全ウェイポイントをまとめて削除できます。


### 4. Auto Capture
#### 自己位置からの自動追加
- パネルの **Auto Capture** セクションで以下を設定します：
  - `Auto Δd` : 自動追加するウェイポイント間の最小距離
  - `Topic` : 自己位置トピック（例 `/amcl_pose`）
  - `Type` : `PoseWithCovarianceStamped` または `PoseStamped`
- "**Start Auto Capture**" を押すと、移動に合わせて自動でウェイポイントが追加されます。もう一度押すと停止します。

![auto_capture_waypoints_demo](https://raw.githubusercontent.com/wiki/kzm784/waypoint_editor/images/auto_capture_waypoints_demo.gif)

## ライセンス
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

本プロジェクトは Apache License, Version 2.0 のもとで配布されています。
詳細は [LICENSE](LICENSE) ファイルをご覧ください。
