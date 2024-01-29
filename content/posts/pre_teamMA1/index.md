---
title: "011 - vnoidベースの開発例"
date: 2023-1-25
categories: ["過去のイベント結果 & レビュー"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

2023年11月11日に本大会初の試みとなるプレチャレンジが開催されました。  
私たちteamMA1もそのイベントに参加しました。  
今回はそこで発表した内容についてより詳しく説明しようと思います。  
vnoidベースの開発例として、あくまでご参考までに見ていただければと思います。

---

## teamMA1の戦略

私たちは、本年度より改装されたアスレチックコースのうち、まずは低難度コースの踏破を目指します。  
低難度コースは不整地、S字路、階段の３つのエリアに分かれており、  
私たちはジョイスティックで人型ロボットを遠隔操作することでこれらの踏破を目指します。

不整地やS字路に関しては配布されているvnoidを用いれば、苦戦しつつもなんとか踏破できます。  
しかし階段を歩かせるには独自に改良が必要です。  
階段は不整地のようなちょっとした凹凸ではなく、明確な床面の高低差があります。  
よって、人型ロボットはその高低差を意識的に昇り降りしなければいけません。  
そこで本チームでは、まず床面の高低差を検出する機能をvnoidに追加します。  

また、床面の高低差を人型ロボットに伝えるだけでは階段の踏破は難しいと思われます。  
階段には一段ごとに着地可能な領域が明確に定められており、  
そこを踏み外すと歩行を継続できません。  
そこで、高低差を含む床面の着地可能領域を検出する機能を追加します。  
その機能を実現するために、人型ロボットに深度カメラを搭載します。

私たちの基本戦略は、床面上の着地可能領域内に着地足が収まるように、  
ジョイスティックからの入力を制限するというものです。  
この方法で低難度コースの完走を目指します。

なお、開発はUbuntu環境で行います。

---

## LiDARを搭載する

まずは、床面の情報をロボットに伝えるために深度カメラを搭載します。  
深度カメラとは、通常のカメラで撮影される二次元画像に加えて、  
その画像に対応する深度マップ(画像内の物体表面までの視点からの距離を格納したもの)  
を取得するカメラです。
なお、距離センサでも代用可能だと思います。

Choreonoidでは[深度カメラのデバイス型](https://choreonoid.org/ja/manuals/latest/simulation/vision-simulation.html)がRangeCameraとして定義されております。  
このデバイス型から生成される深度カメラのオブジェクトには、  
深度マップ付きの二次元画像を取得する機能が備わっています。  
そのため、ロボットモデルに深度カメラを取りつけさえすれば、  
すぐにシミュレーションで深度カメラを試すことができます。

ロボットに深度カメラを搭載する方法について説明します。  
vnoidのロボットモデルは、  
'vnoid/model/sample_robot/sample_robot_ver2.body'  
に記述されています。
[ChoreonoidのBodyファイルチュートリアル](https://choreonoid.org/ja/manuals/latest/handling-models/modelfile/modelfile-newformat.html)を参考に、  
以下のようにロボットの頭リンク(HEAD_P)に深度カメラ(CameraBody)を搭載しました。

'''
  -
    name: HEAD_P
    parent: HEAD_Y
    
    ...

  -
    name: CameraBody
    parent: HEAD_P
    jointType: fixed
    mass: 0
    centerOfMass: [0, 0, 0]
    inertia: [1, 0, 0, 
              0, 1, 0, 
              0, 0, 1]
    elements: 
      Transform: 
        translation: [0.10, 0, 0.0]
        rotation: [[0.0, 0.0, 1.0, -90.0], [1.0, 0.0, 0.0, -30.0]]
        elements: 
          -
            type: Shape
            geometry: {type: Box, size: [0.25, 0.1, 0.1]}
            apperarance: &Rail_apperance
              material: 
                diffuseColor: [0.1, 0.1, 0.8]
                specularColor: [0.5, 0.5, 0.5]
                shininess: 0.6
          -
            translation: [0.0, 0.05, 0.0]
            type: Shape
            geometry: 
              type: Cylinder
              radius: 0.03
              height: 0.02
            appearance: 
              material: 
                diffuseColor: [0.8, 0.8, 0.8]
                specularColor: [0.5, 0.5, 0.5]
                shininess: 0.6
          -
            type: Camera
            name: RangeCamera
            id: 0
            translation: [0.0, 0.0, 0.0] 
            rotation: [1.0, 0.0, 0.0, 60.0]
            format: COLOR_DEPTH
            lens_type: "NORMAL"
            on: true
            width: 640
            height: 480
            field_of_view: 60
            near_clip_distance: 0.4
            far_clip_distance: 4.5
            frame_rate: 30
'''

---

## PCL(Point Clound Library)の導入

撮影した床面の深度マップ付きの二次元画像から、  
視点座標基準の床面の三次元点群を抽出することができます。  
この三次元点群を処理するためのライブラリとして、  
PCL(Point Clound Library)を導入します。  
Ubuntu環境であれば、以下のコマンドをターミナル上で入力することでインストールできます。

'''
sudo apt install libpcl-dev
sudo apt install pcl-tools
'''

---

## コントローラの枠組み

私たちは、下図のように人型ロボットのコントローラを拡張しました。  
{{<figure src="./class_structure.png" class="center" alt="beta" width="100%">}}

VnoidSampleControllerは、Choreonoidのサンプル用に設計された[SimpleController](https://choreonoid.org/ja/manuals/1.5/simulation/howto-implement-controller.html)  
を継承したクラスです。

VnoidSampleControllerからMyCameraにスキャン指令(GoundScan)を送ると、  
MyCameraから床面の着地可能領域が返ってくるという設計にしました。

また、VnoidSampleControllerからMyRobotに制御指令(control)を送ると、  
床面上の着地可能領域内に着地足が収まるように着地位置が計画され、  
それを追従できるような歩行安定化制御をします。

'vnoid/controller/sample_controller/main.cpp'の中身は以下のように変更しました。

'''cpp {linenos=inline}
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "myrobot.h"
#include "mycamera.h"

using namespace cnoid;
using namespace cnoid::vnoid;

class VnoidSampleController : public SimpleController{
public:
	MyRobot*  robot;
    MyCamera* camera;
    Joystick joystick;
    bool PreButtonState;
    int count;

public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
        camera = new MyCamera();
        camera->Init(io);
        count = 0;

		robot = new MyRobot();
		robot->Init(io);

		return true;
	}

	virtual bool control()	{
        joystick.readCurrentState();
        bool ButtonState = joystick.getButtonState(Joystick::A_BUTTON);
        if (ButtonState && !PreButtonState) {
            printf("push A_BUTTON\n");
            #endif
            camera->GroundScan();
        }
        PreButtonState = ButtonState;
        
		robot->Control();
        count++;
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
'''

現状はジョイスティックのAボタンを押してMyCameraのGroundScan()を呼び出す仕様となっています。

---

## 深度カメラ視野内の平面検出

PCLを使って、取得した三次元点群のうち、平面を構成している点集合を抽出します。  
'vnoid/src/mycamera.cpp'を次のように作成しました。  
'''cpp {linenos=inline}
#include "mycamera.h"

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    #ifdef _WIN64
    cnoid::vnoid::Debug::Out("viewerOneOff\n");
    #else
    printf("viewerOneOff\n");
    #endif
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    #ifdef _WIN64
    cnoid::vnoid::Debug::Out("viewerPsyco\n");
    #else
    printf("viewerPsyco\n");
    #endif
}

namespace cnoid {
namespace vnoid {

MyCamera::MyCamera() {
    timeStep = 1.0;
}

void MyCamera::Init(SimpleControllerIO* io) {
    // enable camera
    cameras << io->body()->devices();
    for (size_t i = 0; i < cameras.size(); ++i) {
        Device* camera = cameras[i];
        io->enableInput(camera);
        
        #ifdef _WIN64
        OPD("Device type: %s, ", camera->typeName());
        OPD("id: %s, ", camera->id());
        OPD("name: %s.\n", camera->name());
        #else
        printf("Device type: %s, ", camera->typeName());
        printf("id: %d, ", camera->id());
        printf("name: %s.\n", camera->name());
        #endif

    }

    timeCounter = 0.0;
    timeStep = io->timeStep();
}

void MyCamera::GroundScan() {
    // get cameras
    // when there are several cameras
    for (size_t i = 0; i < cameras.size(); i++) {
        RangeCamera* camera = cameras[i];
        // describe here
    }
    // only one camera
    RangeCamera* camera = cameras[0];

    // Get an image of the current scene
    const Image& RangeImage = camera->constImage();
    // Save an image of current scene
    RangeImage.save("pointcloud.png");
    #ifdef _WIN64
    OPD("save image.\n");
    #else
    printf("save image.\n");
    #endif
    
    // width and height of this image
    const int width = RangeImage.width();
    const int height = RangeImage.height();
    // get color data of this image
    const unsigned char* pixels = RangeImage.pixels();

    // point cloud variable declaration
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // initialize point cloud
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // Stores values (coordinates, color) for each point in a point cloud
    std::size_t i = 0;
    for (const auto& e : camera->constPoints()) {
        //pcl::PointXYZ& point = cloud->points[i];
        pcl::PointXYZRGB& point = cloud->points[i];

        // X, Y, Z
        point.x = e(0);
        point.y = e(1);
        point.z = e(2);

        // color(R, G, B)
        //point.r = pixels[3 * i + 0];
        //point.g = pixels[3 * i + 1];
        //point.b = pixels[3 * i + 2];

        point.r = 255;
        point.g = 255;
        point.b = 255;

        ++i;
    }

    // create the model coeeficients object
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);

    // create the inliers object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // create the SACSegmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // optional setting
    seg.setOptimizeCoefficients(true);

    // madatory settings
    seg.setModelType(pcl::SACMODEL_PLANE);  // detect plane
    seg.setMethodType(pcl::SAC_RANSAC); // use RANSAC algorithm
    seg.setDistanceThreshold(0.01); // how close a point must be to the ,model in order to be consider an inlier

    // input cloud to segmentation
    seg.setInputCloud(cloud);

    // plane segmentation
    seg.segment(*inliers, *coeffs);

    // coloring plane segments
    for (size_t i = 0; i < inliers->indices.size(); i++){
        cloud->points[inliers->indices[i]].r = 255;
        cloud->points[inliers->indices[i]].g = 0;
        cloud->points[inliers->indices[i]].b = 0;
    }

    // make a viewer of point cloud
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(cloud);

    
    // set the thread that called at once in visualization
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // set the thread running while visualization
    viewer.runOnVisualizationThread(viewerPsycho);

}

}  // namespace vnoid
}  // namespace cnoid
'''

---

## 平面を構成する三次元点群を視点座標系から支持足基準座標系に変換する

検出された平面上の三次元点群が得られました。
これらをすべて視点座標系から支持足座標系へ変換します。


---

## 着地可能領域の検出

支持足座標系における検出平面上の三次元点群が得られました。
着地可能領域とは、これら点群の


---

## まとめ・次回予告



次回： [012 - 歩行制御器](https://koomiy.github.io/posts/stepping_controller/)
