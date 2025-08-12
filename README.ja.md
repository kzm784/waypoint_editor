[English](README.md) | [日本語](README.ja.md)

[![Build on Humble](https://github.com/kzm784/waypoint_editor/actions/workflows/main.yml/badge.svg)](https://github.com/kzm784/waypoint_editor/actions/workflows/main.yml)

# Waypoint Editor

![demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/waypoint_editor_demo.gif)


## 目次
- [概要](#概要)
- [開発環境](#開発環境)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)
- [ライセンス](#ライセンス)


## 概要
このパッケージは、ナビゲーションで使用するウェイポイント（Waypoint）を、2次元地図を見ながら直感的に編集・保存できるツールです。  
編集したウェイポイントは **CSV形式**で保存が可能です。


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

### 2. 2Dマップの読み込み  
- Nav2 の `nav2_map_server` を利用して `.yaml` 形式の2Dマップを読み込みます。  
- RViz2 画面右下のパネルで "**Load Map**" をクリックし、使用したい `.yaml` ファイルを選択してください。

![load_map_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_2d_map_demo.gif)


### 3. ウェイポイントの追加  
- RViz2 画面上部のツールバーから "**Add Waypoint**" を選択します。  
- 地図上でドラッグ＆ドロップすることで、位置と向きを指定して新しいウェイポイントを追加できます。  
- 追加されたウェイポイントは：
  - **ドラッグで移動・回転**が可能
  - **右クリックでメニュー表示**から削除や編集が可能

![adding_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/Adding_waypoints_demo.gif)


### 4. ウェイポイントの保存  
- RViz2 画面右下のパネルで "**Save WPs**" を選択し、保存したいファイル名を入力することで、編集内容を **CSV形式**で保存できます。

![saving_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/saving_waypoints.gif)


### 5. ウェイポイントの読み込み  
- "**Load WPs**" ボタンから、過去に保存した `.csv` ファイルを読み込み、ウェイポイントの再編集が可能です。

![loading_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_waypoints.gif)


## ライセンス
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

本プロジェクトは Apache License, Version 2.0 のもとで配布されています。
詳細は [LICENSE](LICENSE) ファイルをご覧ください。
