---
title: "011 - vnoidベースの開発例その１"
date: 2024-01-25
categories: ["vnoidベースの開発"]
menu: main
---

---

## 今回の内容

2023年11月11日に本大会初の試みとなるプレチャレンジが開催されました。  
私たちteamMA1もそのイベントに参加しました。  
今回はそこで発表した内容についてより詳しく説明しようと思います。  
vnoidベースの開発例として、あくまでご参考までに見ていただければと思います。

---

## teamMA1の戦略

私たちは、本年度より改装されたアスレチックコースのうち、  
まずは低難度コースの踏破を目指します。  
低難度コースは不整地、S字路、階段の３つのエリアに分かれており、  
私たちはジョイスティックで人型ロボットを遠隔操作することでこれらの踏破を目指します。

不整地やS字路に関しては配布されているvnoidを用いれば、  
苦戦しつつもなんとか踏破できます。  
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

## 深度カメラを搭載する

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

ロボットへの深度カメラの搭載例を紹介します。  
vnoidのロボットモデルは、  
`vnoid/model/sample_robot/sample_robot_ver2.body`  
に記述されています。  
私たちは、[ChoreonoidのBodyファイルチュートリアル](https://choreonoid.org/ja/manuals/latest/handling-models/modelfile/modelfile-newformat.html)を参考に、  
以下のようにロボットの頭リンク(HEAD_P)に深度カメラ(CameraBody)を搭載しました。

```yaml
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
```

このようにすると、次の画像のように深度カメラが取り付けられます。  
{{<figure src="./camera.png" class="center" alt="beta" width="60%">}}

---

## PCL(Point Clound Library)の導入

Choreonoidでは、撮影した床面の深度マップ付きの二次元画像から、  
視点座標系における床面の三次元点群を抽出することができます。

この三次元点群を処理するためのライブラリとして、  
PCL(Point Clound Library)を導入します。

Ubuntu環境であれば、  
以下のコマンドをターミナル上で入力することでインストールできます。

```
sudo apt install libpcl-dev
sudo apt install pcl-tools
```

---

## コントローラの枠組み

私たちは、下図のように人型ロボットのコントローラを拡張しました。  
{{<figure src="./class_structure.png" class="center" alt="beta" width="100%">}}

VnoidSampleControllerはChoreonoidのサンプル用に設計された  
[SimpleController](https://choreonoid.org/ja/manuals/1.5/simulation/howto-implement-controller.html)を継承したクラスです。

VnoidSampleControllerからMyCameraにスキャン指令(GoundScan)を送ると、  
MyCameraから床面の着地可能領域が返ってくるという設計にしました。

また、VnoidSampleControllerからMyRobotに制御指令(control)を送ると、  
床面上の着地可能領域内に着地足が収まるように着地位置が計画され、  
それを追従できるような歩行安定化制御をするように設計します。

`vnoid/controller/sample_controller/main.cpp`の中身は以下のように変更しました。

```cpp {linenos=inline}
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
            camera->GroundScan();
        }
        PreButtonState = ButtonState;
        
		robot->Control();
        count++;
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
```

現状、ジョイスティックのAボタンを押すと、  
MyCameraのGroundScan()が呼び出される仕様となっています。

---

## 深度カメラ視野内の平面検出

PCLを使って、取得した三次元点群のうち、平面を構成している点群を抽出します。  
そのために`vnoid/src/mycamera.h, mycamera.cpp`を次のように作成しました。

```cpp {linenos=inline}
#pragma once

#include <cnoid/SimpleController>
#include <cnoid/EigenTypes>
#include <cnoid/RangeCamera>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

namespace cnoid {
namespace vnoid {

class MyCamera : RangeCamera
{
public: 
    DeviceList<RangeCamera> cameras;
    double timeCounter;
    double timeStep;

public: 
    virtual void Init(SimpleControllerIO* io);
    virtual void GroundScan();

    MyCamera();

};

}  // namespace vnoid
}  // namespace cnoid
```

```cpp {linenos=inline}
#include "mycamera.h"

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    printf("viewerOneOff\n");
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    printf("viewerPsyco\n");
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
        
        printf("Device type: %s, ", camera->typeName());
        printf("id: %d, ", camera->id());
        printf("name: %s.\n", camera->name());
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
    printf("save image.\n");
    
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
```

`mycamera.cpp`のプログラム概要を説明します。  
まず、Init関数で、bodyファイルにおいて定義した台数分だけ  
カメラデバイス(cameras)を認識させます。  
今回一つの深度カメラのみを用いるので、  
45行目でRangeCameraのオブジェクト(camera)を、
cameras[0]に設定します。

48行目でカメラのシャッターを切ります。

54~57行目で、画像の幅や高さ、ピクセル数などを取得します。  
これには、bodyファイルにて設定したパラメータが反映されます。

61行目で、pcl::PointCloud\< pcl::PointXYZRGB \>::Ptr型として、  
cloudオブジェクトを生成します。  
これは、色つき三次元点群を格納するためのオブジェクトです。  
63~66行目で、cloudにも撮影画像のサイズ情報を与えます。

68行目~84行目で、cloudに色つき三次元点群を格納します。  
camera->constPoints()は、深度マップを三次元点群に変換して返す関数で、  
その点を格子点ごとにcloud->points[i]に格納します。  
後の平面検出のために、RGB値は白色となるように設定します。

87行目以降は、[RANSACアルゴリズム](https://tech-deliberate-jiro.com/python-ransac/)を用いて、  
三次元点群から平面を検出するためのプログラムです。

87行目で、平面モデル(方程式)の係数を定義します。

90行目で、インライアを定義します。  
インライアとは、平面モデルから外れていないと推定されるデータのことです。

93行目で、三次元点群を分割するためのオブジェクトを定義します。

96行目で、外れ値の存在を前提として最適化するように設定します。

99行目で、RANSACのモードを平面検出に設定します。

101行目で、インライアとして判定する際の閾値を設定します。

104行目で、入力三次元点群をセットします。

107行目で平面検出をします。

110~114行目で、インライア判定された三次元点群のみを赤色に塗ります。  
こうすることで、平面として検出された三次元点群のみ赤色で表示されます。

117~125行目は、PointCloudViewerを呼び出して、  
三次元点群を表示するためのプログラムです。

実際にプログラムを実行すると、  
ビューワーが起動して次のような画像が表示されます。  
{{<figure src="./scanned_ground.png" class="center" alt="beta" width="80%">}}  
期待通り、平面として検出された三次元点群のみ赤色で表示されています。

---

## 平面上の三次元点群を支持足基準座標系に変換

PCLにより視点座標系における検出平面上の三次元点群$\boldsymbol{{}^Cp_G}$が得られました。  
これらをすべて視点座標系から支持足座標系へ変換します。  
これには[順運動学計算](https://koomiy.github.io/posts/fk_solver/)を用いるので、  
リンク先の解説と合わせて読んでいただければと思います。

まず、視点座標系における検出平面の三次元点群を、ベースリンク座標系に変換します。  
$$ \boldsymbol{{}^Bp_G} = \boldsymbol{{}^Bp_H} + \boldsymbol{{}^BR_H} (\boldsymbol{{}^Hp_C} + \boldsymbol{{}^HR_C} \boldsymbol{{}^Cp_G}) $$
ここで、$B$はベースリンク座標、$H$は頭リンク座標、  
$C$はカメラ視点座標、$G$は床の平面上の各点座標を意味します。  
頭リンクから見たカメラ視点の位置$\boldsymbol{{}^Hp_C}$や姿勢$\boldsymbol{{}^HR_C}$は搭載時に自ら設定するので既知です。  
また、ベースリンクから頭リンクまでの関節は歩行中にほとんど回転しないので、  
$\boldsymbol{{}^Bp_H} = \boldsymbol{I}$として、上式は次のように簡単に書けます。  
$$ \boldsymbol{{}^Bp_G} = \boldsymbol{{}^Bp_H} + \boldsymbol{{}^Hp_C} + \boldsymbol{{}^HR_C} \boldsymbol{{}^Cp_G} $$

求めたい足座標系における検出平面上の三次元点群$\boldsymbol{{}^Fp_G}$を用いると、  
ベースリンク座標系における検出平面上の三次元点群$\boldsymbol{{}^Bp_G}$は次のようにも書けます。
$$ \boldsymbol{{}^Bp_G} = \boldsymbol{{}^Bp_A} + \boldsymbol{{}^BR_A} (\boldsymbol{{}^Ap_F} + \boldsymbol{{}^AR_F} \boldsymbol{{}^Fp_G}) $$
ここで、$A$は足首リンク座標、$F$は足裏中心座標を意味します。  
vnoidには、ロボットの関節角から、  
ベースリンク座標系における足首の位置$\boldsymbol{{}^Bp_A}$・姿勢$\boldsymbol{{}^BR_A}$を計算する  
順運動学計算器(CompLegFK)が`vnoid/src/fksolver.cpp`に用意されています。  
よって、上式において、$\boldsymbol{{}^Bp_A}$や$\boldsymbol{{}^BR_A}$は既知です。  
また、$\boldsymbol{{}^Ap_F}$は足首から足裏中心までの相対位置ですが、  
単にz軸方向に足の厚み分だけオフセットすることを表現するので既知です。  
さらに、足首リンク座標と足裏中心座標の姿勢は一致するので、  
$\boldsymbol{{}^AR_F} = \boldsymbol{I}$とできます。
よって、上式は次のように簡単に書き直せます。
$$ \boldsymbol{{}^Bp_G} = \boldsymbol{{}^Bp_A} + \boldsymbol{{}^BR_A} (\boldsymbol{{}^Ap_F} + \boldsymbol{{}^Fp_G}) $$


以上の簡単化した二式を用いて、  
視点座標における検出平面上の三次元点群を足座標系に変換します。  
$$ \boldsymbol{{}^Fp_G} = \boldsymbol{{}^BR_A}{}^T (\boldsymbol{{}^Bp_H} + \boldsymbol{{}^Hp_C} + \boldsymbol{{}^HR_C \boldsymbol{{}^Cp_G}} - \boldsymbol{{}^Bp_A}) - \boldsymbol{{}^Ap_F} $$

支持側の足で以上の計算をすることで、  
支持足を基準とした検出平面上の三次元点群$\boldsymbol{{}^Fp_G}$が得られます。

---

## 着地可能領域の検出

支持足座標系における検出平面上の三次元点群が得られました。  
着地可能領域を、これら点群の[凹包](https://postgis.net/docs/manual-3.4/ja/ST_ConcaveHull.html)として定義します。

現在は、PCLの[ConcaveHullクラス](https://pointclouds.org/documentation/classpcl_1_1_concave_hull.html)を用いて、  
凹包の頂点の集合を抽出できないかを模索している段階です。

---

## 着地可能領域内で着地位置を計画

着地可能領域に含まれる支持足基準の高低差情報を用いて、  
自動的に階段の昇降が可能となるように着地高さの計画を行います。

また、着地可能領域外を踏めば、壁にぶつかったり、  
床を踏み外したりといったことが起こり得ます。  
そこで、操縦者の入力により領域外に出てしまうなら、  
その入力は受け付けないという危険防止装置を組み込みます。

こうすることで、着地可能領域内であれば  
操縦者が自由に着地位置を計画できるようなシステムを構築できます。

領域の内外判定には[Crossing Number Algorithm](https://www.nttpc.co.jp/technology/number_algorithm.html)を用いる予定です。

---

## まとめ・次回予告

今回は、私たちのチームの開発内容を紹介しました。  
私たちは、深度カメラを用いて着地できる床上の領域を検出し、  
その中での歩行を遠隔操作で実現できるようなシステムを開発しております。

vnoidベースの開発の一例として見ていただけたなら幸いです。

次回： [012 - 歩行制御器](https://koomiy.github.io/posts/stepping_controller/)
