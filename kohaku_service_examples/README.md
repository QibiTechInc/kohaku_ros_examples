# 概要

Kohakuの提供しているサービスを使った例を示すためのROSパッケージです。


# Nodes

それぞれのサンプルは、すべて右腕の動作を確認するものとなっています。

## reset_alarm_example_node
このサンプルでは、すべてのジョイントに発生したアラームをリセットしています。

## servo_allon_example_node
このサンプルでは、すべてのジョイントのサーボの電源をONにしています。

## servo_alloff_example_node
このサンプルでは、すべてのジョイントのサーボの電源をOFFにしています。

## set_joint_trajectory_example_node
このサンプルでは、位置制御によるジョイントの移動を確認することができます。

## mode_change_pos2cur_example_node
このサンプルでは、位置制御から電流制御への切替え、および電流制御時の腕の動きを確認することができます。


# 動作確認方法

サンプルはそれぞれ以下のように実行できます。前提として kohaku_ros_driver の kohaku_control.launch が起動している必要があります。

``` bash
$ rosrun kohaku_service_examples kohaku_reset_alarm.py
```

``` bash
$ rosrun kohaku_service_examples kohaku_servo_allon.py
```

``` bash
$ rosrun kohaku_service_examples kohaku_servo_alloff.py
```

``` bash
$ rosrun kohaku_service_examples kohaku_set_joint_trajectory.py
```

``` bash
$ rosrun kohaku_service_examples kohaku_mode_change_pos2cur.py
```
