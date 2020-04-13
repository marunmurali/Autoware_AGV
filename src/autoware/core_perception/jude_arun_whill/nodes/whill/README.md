# WHILL
WHILL Model AおよびCのドライバと、imu、ロータリエンコーダのドライバ、実験用のlaunchファイルや設定ファイルがあるパッケージです。

## WHILL_Command_Model_A.py & WHILL_Command_Model_A.py
WHILLの通信を行うクラスです。基本変更しなくていいと思います。

## cmd_vel_multiplexer
複数のcmd_velに優先順位をつけて出力します。同時に複数のcmd_velが来た場合は優先度が高いものだけを出力し、ほかは捨てます。

### Subscribed
* ~/cmd_vel_priority_high(geometry_msgs/Twist)
* ~/cmd_vel_priority_mid(geometry_msgs/Twist)
* ~/cmd_vel_priority_low(geometry_msgs/Twist)

### Published
* ~/cmd_vel_out(geometry_msgs/Twist)

### Param
* hold_sec(float)  
優先度が高い入力の後、hold_sec秒以内に来た優先度の低い入力は無視します。

## depth_camera_calibrator
デプスカメラの位置を正しく設定するためのプログラムです。あんま使えないです。具体的にはポイントクラウドから(x,y,z)=(1,0,0)における法線を表示します。
### Subscribed
* /royale_camera_driver/point_cloud(sensor_msgs/PointCloud2)  

### Published
none

### Param
* frame_id(std::string, default: "base_link")
* subscribed_topic(std::string, default: "/royale_camera_driver/point_cloud")
* search_radius(float, default: 0.3)  
法線推定に使う点の範囲です。

## feedback_controller
cmd_velに対する比例フィードバックを行います。WHILLに送るのは速度指令値なので、指令値をフィードバックするのはあまり意味がないかと思うので使わなくていいと思います。参考程度においておきます。

## gyro_odometry
エンコーダによるオドメトリ計算用プログラムです。DACSのカウンタと通信して普通の対向二輪のオドメトリを出力します。エンコーダ単体でも機能しますが、角速度に関してはIMUの方が精度が良いので角速度だけIMUからのトピックを受け取って使うことができるようにしています。

### Subscribed
* ~/imu(sensor_msgs/Imu)

### Published
* ~/odom(nav_msgs/Odometry)  
オドメトリ型のオドメトリ情報です。TF型で出しているものと同じです。基本はTFを使うと思いますが、Odometry型のトピックを必要とする他ノードもいるかも知れないのでつけています。
* ~/speed(std_msgs/Float32)  
前進速度をスカラ値で出します。
* ~/vel_out(geometry_msgs/Twist)  
前進速度と回転速度をTwist型で出します。
* TF("odom" to "base_link")  
オドメトリ情報です。ノードを起動した地点から積分計算したローカル位置です。

### Parameters
* PARENT_FRAME(std::string, "odom")
* CHILD_FRAME(std::string, "base_link")

## imu_node
東京航空計器のIMUを使う時のドライバプログラムです。センサと通信してIMUトピックを出力します。最近XSENSに変えたのでこちらは使わないと思います。参考までに残しておきます。

## odom_plot.py
オドメトリで計算したローカル位置を各時刻でプロットしてファイルに保存するプログラムです。ファイルの保存場所はプログラム中にあります。要確認。

### Subscribed
* /odom(nav_msgs/Odometry)

## scan_test
scan型のトピックを2回に1回だけ出力するサンプルプログラムです。用途は謎ですがコードが簡単なので参考にはなるかと思います。

## static_tf
値固定でTFを出力するサンプルプログラムです。TF出力の参考になると思います。


## whill_node_Model_A
This node send/recieve "WHILL command" to/from WHILL.
WHILL command contains the data below.

### send
* joystick value horizontal (-100 ~ 100)
* joystick value vartical (-100 ~ 100)
* speed profile
* Start/Stop sending data
* Move seat

### recieve
* acceleration (XYZ axis)
* angular rate (XYZ axis)
* joystick value
* battery power and current
* speed
* seat sensor
* is power on
* is seat back
* error
* indicator position

And, about node
### subscribed
* /cmd_vel (geometry_msgs::Twist)  
WHILLに送る速度の目標値です。

### published
* ~/joystick (geometry_msgs::Vector3)  
WHILLのジョイスティックの値です。
* /raw_speed (std_msgs::Float32)  
WHILLが内部で計算した速度です。精度は不明です。
* /whill_data(whill_data)  
WHILLとの通信で得られる様々なデータを見る用です。独自定義のメッセージ型です。

### Parameters
* port(string, "/dev/whill/whill")  
* drive_mode(string, "normal")  
速度パターンが変更できます。通常はnormalで良いと思います。他は遊び用です。  
normal→普通、high_speed→高速モード、maximum→最大加速&最大速度モード
