// ROSメインヘッダーファイルのインクルード
#include <ros/ros.h>

// std_msgsのFloat32MultiArray型のメッセージを使用するので、そのヘッダをインクルード
// Float32MultiArray型はfloat型の配列
#include <std_msgs/Float32MultiArray.h>

// 配信者ノードのメイン関数
int main(int argc, char **argv)
{

  // ノード名の初期化
  ros::init(argc, argv, "launch_pub_node");

  // ノードハンドルを宣言
  ros::NodeHandle nh;

  // Publiserの宣言
  // std_msgsのFloat32MultiArray型
  // トピック名　"numbers"
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("numbers", 10);

  // ループ周期の設定
  // 10Hz = 0.1秒間隔
  ros::Rate loop_rate(10);

  // メッセージに使用する変数の宣言
  int data_size = 4;
  float numbers[data_size] = {1.8, 2.7, 6.3, 8.5};

  // ros::ok()はROSの動作が正常であるならtrueを返す関数
  while (ros::ok())
  {
    // Float32MultiArray型でmsg変数を宣言する。
    std_msgs::Float32MultiArray msg;

    // 配列をリサイズ
    msg.data.resize(data_size);

    // 配列に値を代入して表示
    // ROS_INFOというROS関数を使用して、msg変数の配列データをを表示する。
    for (int i = 0; i < data_size; i++)
    {
      msg.data[i] = numbers[i];
      ROS_INFO("numbers[%i]:%.2f", i, msg.data[i]);
    }

    // メッセージを配信する
    pub.publish(msg);
    ROS_INFO("Published msg!");

    // 上で定められたループサイクルになるように、スリープに入る
    loop_rate.sleep();
  }

  return 0;
}
