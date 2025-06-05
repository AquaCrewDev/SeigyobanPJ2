//SwitchRobo.bodyファイルを制御する
#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <iostream>

using namespace cnoid;

class AutoBoxController : public SimpleController
{
    Link* box;      // Boxリンクを指すポインタ
    Link* smallBox; // SmallBoxリンクを指すポインタ
    Link* robo;     // Roboリンクを指すポインタ
    double q_ref_box;       // Boxリンクの目標角度
    double q_prev_box;      // 前回のBoxリンクの角度
    double q_initial_box;   // 初期のBoxリンクの角度
    double q_ref_smallBox;  // SmallBoxリンクの目標角度
    double q_prev_smallBox; // 前回のSmallBoxリンクの角度
    double q_initial_smallBox; // 初期のSmallBoxリンクの角度
    double q_ref_robo;      // Roboリンクの目標角度
    double q_prev_robo;     // 前回のRoboリンクの角度
    double dt;  // 制御の時間ステップ
    Joystick joystick;    // ジョイスティック
    bool actionTriggered = false;  // アクションがトリガーされたかどうか
    int state = 0;    // 現在の状態を示す変数
    double timer = 0.0;   // タイマー
    double moveDistance = 0.0;  // 移動距離

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        // Box、SmallBox、Roboリンクを取得
        box = io->body()->link("Box");
        smallBox = io->body()->link("SmallBox");
        robo = io->body()->link("Robo");

        // デバイスやリンクが見つからない場合のエラーチェック
        if (!box || !smallBox || !robo) {
            std::cerr << "Error: Link(s) not found!" << std::endl;
            std::cerr << "Box: " << box << ", SmallBox: " << smallBox << ", Robo: " << robo << std::endl;
            return false;
        }
        
        // リンクのアクチュエーションモードをトルク制御に設定
        box->setActuationMode(Link::JOINT_TORQUE);
        smallBox->setActuationMode(Link::JOINT_TORQUE);
        robo->setActuationMode(Link::JOINT_TORQUE);
        
        // リンクの入出力を有効化
        io->enableIO(box);
        io->enableIO(smallBox);
        io->enableIO(robo);
        
        // 初期値の設定
        q_ref_box = q_prev_box = q_initial_box = box->q();
        q_ref_smallBox = q_prev_smallBox = q_initial_smallBox = smallBox->q();
        q_ref_robo = q_prev_robo = robo->q();

        // 制御の時間ステップを取得
        dt = io->timeStep();

        std::cout << "Initialization complete." << std::endl;
        return true; // 初期化が成功したことを示す
    }

    virtual bool control() override
    {
        static const double P = 200.0;    // 比例ゲイン
        static const double D = 50.0;     // 微分ゲイン
        // ジョイスティックの現在の状態を読み取る
        joystick.readCurrentState();

        // ボタンの状態をコンソールに出力
        for (int i = 0; i < joystick.numButtons(); ++i) {
            if (joystick.getButtonState(i)) {
                std::cout << "Button " << i << " pressed." << std::endl;
            }
        }

        // アクションがトリガーされていない場合、特定のボタンが押されたかどうかをチェック
        if (!actionTriggered) {
            if (joystick.getButtonState(0)) {
                actionTriggered = true;   // アクションがトリガーされたことを設定
                state = 1;  // 状態を1に設定
                timer = 0.0;    // タイマーをリセット
                moveDistance = -0.00025;  // 移動距離を設定
            } else if (joystick.getButtonState(1)) {
                actionTriggered = true;
                state = 1;
                timer = 0.0;
                moveDistance = -0.0004;
            } else if (joystick.getButtonState(3)) {
                actionTriggered = true;
                state = 1;
                timer = 0.0;
                moveDistance = -0.0001;
            }
        }

        // アクションがトリガーされた場合の状態遷移
        if (actionTriggered) {
            switch (state) {
                case 1: // Boxリンクを右に移動
                    q_ref_box += moveDistance;   // 目標角度を更新
                    if (timer >= 1.0) {
                        state = 2;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 2: // 1秒停止
                    if (timer >= 1.0) {
                        state = 3;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 3: // SmallBoxリンクを左に移動
                    q_ref_smallBox = 0.005;   // 目標角度を更新
                    if (timer >= 1.0) {
                        state = 4;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 4: // 1秒停止
                    if (timer >= 1.0) {
                        state = 5;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 5: // Roboリンクを45度回転
                    q_ref_robo = 45.0 * (M_PI / 180.0);  // 目標角度を45度に設定
                    if (timer >= 1.0) {
                        state = 6;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 6: // 1秒停止
                    if (timer >= 1.0) {
                        state = 8;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 8: // SmallBoxリンクを初期位置に戻す
                    q_ref_smallBox = q_initial_smallBox;    // 目標角度を初期角度に設定
                    if (std::abs(q_ref_smallBox - smallBox->q()) < 1e-6) {
                        state = 7;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 7: // Roboリンクの回転をリセット
                    q_ref_robo = 0.0;   // 目標角度を0に設定
                    if (timer >= 1.0) {
                        state = 9;  // 次の状態に移行
                        timer = 0.0;    // タイマーをリセット
                    }
                    break;
                case 9: // Boxリンクを初期位置に戻す
                    q_ref_box = q_initial_box;    // 目標角度を初期角度に設定
                    if (std::abs(q_ref_box - box->q()) < 1e-6) {
                        actionTriggered = false;   // アクションを終了
                    }
                    break;
            }
            timer += dt; // タイマーを更新
        }

        // Boxリンクの制御
        double q_box = box->q();   // 現在の角度を取得
        double dq_box = (q_box - q_prev_box) / dt;  // 角速度を計算
        box->u() = P * (q_ref_box - q_box) + D * (-dq_box);   // PD制御を適用
        q_prev_box = q_box;   // 前回の角度を更新

        // SmallBoxリンクの制御
        double q_smallBox = smallBox->q();   // 現在の角度を取得
        double dq_smallBox = (q_smallBox - q_prev_smallBox) / dt;  // 角速度を計算
        smallBox->u() = P * (q_ref_smallBox - q_smallBox) + D * (-dq_smallBox);   // PD制御を適用
        q_prev_smallBox = q_smallBox;   // 前回の角度を更新

        // Roboリンクの制御
        double q_robo = robo->q(); // 現在の角度を取得
        double dq_robo = (q_robo - q_prev_robo) / dt;  // 角速度を計算
        robo->u() = P * (q_ref_robo - q_robo) + D * (-dq_robo);   // PD制御を適用
        q_prev_robo = q_robo;   // 前回の角度を更新

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AutoBoxController)

