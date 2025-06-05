#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>

#include <iostream>
#include <string>

// ROS用ヘッダー
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
// image transport用
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV用
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cnoid;
using namespace std;

// CameraSampleクラスは、ChoreonoidのSimpleControllerを継承してカメラデバイスを操作し、ROSへ送信する
class CameraSample : public SimpleController
{
    // カメラデバイスへのポインタ
    Camera* camera;
    // カメラデバイスを制御するリンク
    Link* joint_Camera;
    // ログ出力用
    std::ostream* os;
    // モデル全体の参照
    BodyPtr ioBody;
    // シミュレーションの時間ステップ
    double timeStep;
    // 入出力用のインターフェース
    SimpleControllerIO* io;
    
    // ROSノードハンドル
    ros::NodeHandle node;
    // 画像トピック用のイメージトランスポートパブリッシャー
    image_transport::Publisher pub;

    // カメラの解像度
    int camerawidth;
    int cameraheight;

public:
    // ROSのパブリッシャーを初期化
    virtual bool configure(SimpleControllerConfig* config) override
    {
        // ImageTransportを使用して画像トピックを初期化
        image_transport::ImageTransport it(node);
        pub = it.advertise("out_image_base_topic", 1);
        return true;
    }

    // カメラデバイスの初期化
    virtual bool initialize(SimpleControllerIO* io) override
    {
        cout << "start initialize" << endl;

        this->io = io;
        os = &io->os();

        // カメラデバイスを取得
        camera = io->body()->findDevice<Camera>("RangeCamera");
        // カメラの入力を有効にする
        io->enableInput(camera);
        
        // ボディ情報を取得
        ioBody = io->body();
        // シミュレーションの時間ステップを取得
        timeStep = io->timeStep();
        return true;
    }

    // コントローラ開始時に呼ばれる（通常は処理なし）
    virtual bool start() override
    {
        return true;
    }

    // 各ループで呼ばれるメソッド
    virtual bool control() override
    {
        // カメラ画像を取得
        const Image& image = camera->constImage();
        if (!image.empty()) {
            // カメラの解像度を取得
            const int height = image.height();
            const int width = image.width();
            // OpenCVの画像フォーマットに変換
            cv::Mat sensor_image(height, width, CV_8UC3, const_cast<uchar *>(&image.pixels()[0]));
            // 画像をROSトピックに送信
            pushimage(sensor_image);
        }
        return true;
    }

    // 画像をROSトピックに送信するメソッド
    void pushimage(cv::Mat outputimage)
    {
        // OpenCVのMat形式からROSのImageメッセージ形式に変換
        sensor_msgs::ImagePtr imagemsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", outputimage).toImageMsg();
        // トピックに画像を送信
        pub.publish(imagemsg);
    };
};

// SimpleControllerファクトリを登録
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraSample)

