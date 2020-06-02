// ROSメインヘッダーファイルのインクルード
#include <ros/ros.h>


// std_msgsのFloat32MultiArray型のメッセージを使用するので、
// そのヘッダをインクルード
// Float32MultiArray型はfloat型の配列
#include <std_msgs/Float32MultiArray.h>

// メッセージを受信したときに動作するコールバック関数を定義
// std_msgsのFloat32MultiArray型を受け取る
// コールバック関数内では受け取ったメッセージをmsgとして扱う
void msgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // 受信したメッセージの配列サイズを求める
  // https://qiita.com/SaitoAtsushi/items/ee17466c464fb7a270d2
  float data_size = sizeof(msg->data) / sizeof(msg->data[0]);

  // 配列の要素を合計する
  float sum = 0;
  for(int i = 0; i < data_size; i++)
  {
    sum += msg->data[i];
  }

  // 配列の合計を表示する
  ROS_INFO("received numbers sum: %.2f", sum);

}

// 購読者ノードのメイン関数
int main(int argc, char **argv)
{

  // ノード名の初期化
  ros::init(argc, argv, "launch_sub_node");

  // ノードハンドルを宣言
  ros::NodeHandle nh;

  // Subscriberの宣言
  // std_msgsのFloat32MultiArray型
  // トピック名 "numbers"
  // メッセージを受け取ったらmsgCallback関数を実行する
  ros::Subscriber sub = nh.subscribe("numbers", 10, msgCallback);

  // メッセージを受け取るまで待機
  // 受けとったらコールバック関数を実行
  ros::spin();
  return 0;
}
