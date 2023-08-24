---
title: "005 - 関節を動かしてみる"
date: 2023-07-27
categories: ["choreonoidの練習"]
menu: main
---

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージのプログラムを介して、  
choreonoid上の人型ロボットを動かしてみましょう。

---

## 事前準備

今回は`myrobot.cpp`をメインに作業します。

準備として、まずプログラムの先頭に`#include <cmath>`を追加してください。  
これにより、`M_PI`という変数を円周率 $\pi$ として使用できます。

次に、プログラム10行目を`base_actuation = true;`と設定してください。  
これにより、ベースリンクの位置・姿勢を直接指定するモードに切り替えることができます。

次に、151行目の`Robot::Sense`関数以下に下記のようなプログラムを追加してください。  
```c
void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);
    
    // set base
    base.pos_ref = Vector3(0.0, 0.0, 1.0);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);
    
    ...
    
```
これにより、ベースリンクの位置を原点から高さ1.0mの場所に固定し、  
姿勢は常にまっすぐで変化しないように設定できます。

最後に、今回は`Robot::Sense`、`Robot::Actuate`、  
`timer.Countup`以外の関数を使用しないので、  
`MyRobot::Control`内にある上記４つ以外の関数はすべてコメントアウトしましょう。

これで事前準備完了です！

---

## 例題1: 関節を一つ曲げてみる

試しに右腕のひじ関節を90度曲げてみましょう。  
```c
void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);
    
    // set base
    base.pos_ref = Vector3(0.0, 0.0, 1.0);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);
    
    // set head joint

    // set trunk joint

    // set arm joint
    joint[7].q_ref = -M_PI/2;

    // set leg joint
    
    // calc FK
    //fk_solver.Comp(param, joint, base, centroid, hand, foot);
    
    ...
    
```
vnoidのロボットに使用される関節(joint)には、  
それぞれが識別できるようにidが割り当てられています。  
関節のid割当は、[006 - 順運動学(理論編)](https://koomiy.github.io/posts/fk_solver/)の記事で確認できます。

右腕のひじ関節のidは7番なので、  
7番目の関節の目標角度変位`joint[7].q_ref`を$-\pi/2$にします。

このようにすることで、右腕のひじ関節を90度曲げられます。  
{{<figure src="./right_elbow.gif" class="center" alt="右ひじを曲げる" width="50%">}}

これを応用することで、右ひじ以外にもいろいろな関節を曲げて、  
自分好みのポーズをロボットにとらせることができます！

---

## 例題2: ヨガのポーズ！

例題1を応用して、ヨガのポーズを取らせてみましょう。  
以下のように目標関節角を指定してみます。

```c
void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);
    
    // set base
    base.pos_ref = Vector3(0.0, 0.0, 1.0);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);
    
    // set head joint

    // set trunk joint

    // set arm joint
    joint[4].q_ref = -M_PI/4;
    joint[5].q_ref = -M_PI/6;
    joint[6].q_ref = M_PI/2;
    joint[7].q_ref = -M_PI*(2./3.);
    joint[8].q_ref = M_PI/4;
    joint[9].q_ref = 0.0;
    joint[10].q_ref = -M_PI/2;

    joint[11].q_ref = -M_PI/4;
    joint[12].q_ref = M_PI/6;
    joint[13].q_ref = -M_PI/2;
    joint[14].q_ref = -M_PI*(2./3.);
    joint[15].q_ref = -M_PI/4;
    joint[16].q_ref = 0.0;
    joint[17].q_ref = M_PI/2;

    // set leg joint
    joint[18].q_ref = -M_PI/2;
    joint[19].q_ref = 0.0;
    joint[20].q_ref = -M_PI/3;
    joint[21].q_ref = M_PI*(2./3.);
    joint[22].q_ref = M_PI/6;
    joint[23].q_ref = 0.0;
    
    // calc FK
    //fk_solver.Comp(param, joint, base, centroid, hand, foot);
    
    ...
    
```

すると、次のようなポーズをとってくれます。  
{{<figure src="./yoga_pose.gif" class="center" alt="ヨガのポーズ" width="50%">}}

---

## まとめ・次回予告

今回はロボットにある目標関節角を与えて、いろんなポーズや動作をさせてみました。  
このようにロボットにある関節変位を与えたときの、  
リンクの位置・姿勢を計算することを、順運動学を解くといいます。

次回はその順運動学について解説したいと思います。


次回： [006 - 順運動学](https://koomiy.github.io/posts/fk_solver/)
