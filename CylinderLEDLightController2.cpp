#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Link>
#include <iostream>
#include <cmath>

using namespace cnoid;

class CylinderLEDLightController2 : public SimpleController {
    SpotLight* greenLight;
    SpotLight* yellowLight;
    SpotLight* redLight;
    SpotLight* greenLight1;
    SpotLight* yellowLight1;
    SpotLight* redLight1;
    SpotLight* greenLight2;
    SpotLight* yellowLight2;
    SpotLight* redLight2;
    Link* cylinderJoint;
    Link* cylinderJoint2;
    Link* cylinderJoint3;
    double targetAngle1, targetAngle2, targetAngle3;
    const double torqueThreshold = 0.1; // トルクの閾値
    const double angleThreshold = 35.0; // 角度の閾値（度）
    double timeStep;

public:
    CylinderLEDLightController2();

    virtual bool initialize(SimpleControllerIO* io) override {
        // 各スポットライトを取得
        greenLight = io->body()->findDevice<SpotLight>("GreenLight");
        yellowLight = io->body()->findDevice<SpotLight>("YellowLight");
        redLight = io->body()->findDevice<SpotLight>("RedLight");

        greenLight1 = io->body()->findDevice<SpotLight>("GreenLight1");
        yellowLight1 = io->body()->findDevice<SpotLight>("YellowLight1");
        redLight1 = io->body()->findDevice<SpotLight>("RedLight1");

        greenLight2 = io->body()->findDevice<SpotLight>("GreenLight2");
        yellowLight2 = io->body()->findDevice<SpotLight>("YellowLight2");
        redLight2 = io->body()->findDevice<SpotLight>("RedLight2");

        // 各ジョイントを取得
        cylinderJoint3 = io->body()->link("Cylinder");
        cylinderJoint2 = io->body()->link("Cylinder2");
        cylinderJoint = io->body()->link("Cylinder3");

        // デバイスが見つからない場合のエラーチェック
        if (!greenLight || !yellowLight || !redLight ||
            !greenLight1 || !yellowLight1 || !redLight1 ||
            !greenLight2 || !yellowLight2 || !redLight2 ||
            !cylinderJoint || !cylinderJoint2 || !cylinderJoint3) {
            std::cerr << "SpotLightまたはジョイントが見つかりません。" << std::endl;
            return false;
        }

        // ジョイントのアクチュエーションモードをトルク制御に設定
        cylinderJoint->setActuationMode(Link::JointTorque);
        cylinderJoint2->setActuationMode(Link::JointTorque);
        cylinderJoint3->setActuationMode(Link::JointTorque);
        io->enableIO(cylinderJoint);
        io->enableIO(cylinderJoint2);
        io->enableIO(cylinderJoint3);

        targetAngle1 = cylinderJoint->q();
        targetAngle2 = cylinderJoint2->q();
        targetAngle3 = cylinderJoint3->q();

        timeStep = io->timeStep();

        // 初期状態で全てのスポットライトをオフに設定
        turnOffAllLights();

        return true;
    }

    virtual bool control() override {
        // 現在のジョイント角度を取得
        double jointAngle1 = cylinderJoint->q() * (180.0 / 3.141592653589793);
        double jointAngle2 = cylinderJoint2->q() * (180.0 / 3.141592653589793);
        double jointAngle3 = cylinderJoint3->q() * (180.0 / 3.141592653589793);

        // 外部からの力が加わったかどうかをトルクで判定し、現在の角度を新しい目標角度として設定
        if (std::abs(cylinderJoint->u()) > torqueThreshold) {
            targetAngle1 = cylinderJoint->q();
        }
        if (std::abs(cylinderJoint2->u()) > torqueThreshold) {
            targetAngle2 = cylinderJoint2->q();
        }
        if (std::abs(cylinderJoint3->u()) > torqueThreshold) {
            targetAngle3 = cylinderJoint3->q();
        }

        // PD制御を適用
        double pgain = 0.5;
        double dgain = 0.04;
        applyPDControl(cylinderJoint, targetAngle1, pgain, dgain);
        applyPDControl(cylinderJoint2, targetAngle2, pgain, dgain);
        applyPDControl(cylinderJoint3, targetAngle3, pgain, dgain);

        // 各シリンダーの角度に基づいて対応するライトを制御
        controlLights(jointAngle1, redLight, yellowLight, greenLight);
        controlLights(jointAngle2, redLight1, yellowLight1, greenLight1);
        controlLights(jointAngle3, redLight2, yellowLight2, greenLight2);

        return true;
    }

    void applyPDControl(Link* joint, double targetAngle, double pgain, double dgain) {
        double q = joint->q(); // 現在角度
        double dq = (q - targetAngle) / timeStep; // 現在角速度[rad]

        double u = (targetAngle - q) * pgain + (-dq) * dgain;
        joint->u() = u;
    }

    void controlLights(double angle, SpotLight* redLight, SpotLight* yellowLight, SpotLight* greenLight) {
        if (std::abs(angle) >= angleThreshold) {
            // 緑のライトをオン
            turnOnLight(greenLight);
            // 赤と黄色のライトをオフ
            turnOffLight(redLight);
            turnOffLight(yellowLight);
        } else {
            // 緑のライトをオフ
            turnOffLight(greenLight);
            // 赤と黄色のライトをオフ
            turnOffLight(redLight);
            turnOffLight(yellowLight);
        }
    }

    void turnOnLight(SpotLight* light) {
        if (!light->on()) {
            light->on(true);
            light->notifyStateChange();
            std::cout << light->name() << "がオンになりました。" << std::endl;
        }
    }

    void turnOffLight(SpotLight* light) {
        if (light->on()) {
            light->on(false);
            light->notifyStateChange();
            std::cout << light->name() << "がオフになりました。" << std::endl;
        }
    }

    void turnOffAllLights() {
        turnOffLight(greenLight);
        turnOffLight(yellowLight);
        turnOffLight(redLight);
        turnOffLight(greenLight1);
        turnOffLight(yellowLight1);
        turnOffLight(redLight1);
        turnOffLight(greenLight2);
        turnOffLight(yellowLight2);
        turnOffLight(redLight2);
    }
};

// コンストラクタの定義
CylinderLEDLightController2::CylinderLEDLightController2() {}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CylinderLEDLightController2)

