# Waypoint Editer


## 目次
- [概要](#概要)
- [開発環境](#開発環境)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)

## 概要
ナビゲーションで使用する Waypoint を2次元地図を見ながら編集・保存ができるパッケージです  
編集した Waypoint は CSV 形式で保存が可能です

## 開発環境
- Ubuntu Linux - Jammy Jellyfish 22.04
- ROS 2 Humble Hawksbill

## インストール方法
ターミナルを開き、以下のコマンドでインストールすることができます
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kzm784/waypoint_editer.git
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## 使用方法
- **Waypoint Editer の起動**：  
    - 以下のコマンドを実行し、Waypoint Editer を起動します
        ```bash
        cd ~/ros2_ws
        source install/setup.bash
        ros2 launch waypoint_editer waypoint_editer.launch.py
        ```

- **2D Mapの読み込み**：  
    - 2D Map の読み込みには、Nav2 の nav2_map_server を使用します
    - Rviz2 の画面左下にあるパネルの **Load 2D Map** を選択し、Nav2 起動時に指定する `.yaml` 形式のファイルを選択し、2D Map を読み込みます

- **Waypoint の追加**：  
    - Rviz2の画面上部にあるツール郡の中から、**Add Waypoint** を選択し、画面上で追加したい位置・向きにドラッグ＆ドロップすることで新しい Waypoint を追加することができます
    - 追加されたウェイポイントは、**ドラッグで移動・回転**が可能、**右クリックで各種機能**が選択可能です

- **Waypoint の保存**：  
    - 編集した Waypoint を**CSV形式**で保存することができます
    - Rviz2の画面右下にあるパネルの **Save Waypoints** を選択し、ファイル名を入力することで、編集したウェイポイントを保存することが可能です

- **Waypoint の読み込み**：
    - Rviz2の画面右下にあるパネルの **Load Waypoints** を選択し、`.csv` 形式のファイルを選択することで、以前作成した Waypoint を読み込み、再度編集することが可能です
