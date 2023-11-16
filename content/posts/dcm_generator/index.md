---
title: "009 - 目標DCM計画"
date: 2023-10-05
categories: ["vnoidベースの開発"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、  
目標DCM計画器(footstep_planner::GenerateDCM)について解説します。  

---

## DCMとは

DCM(Divergent Componet of Motion)は、  
その位置に足を踏み出せば自然に停止できる点を意味します。

自然に停止するとは、十分な時間が経過したのちに  
着地位置の真上にロボットの重心が遷移するということです。  
このような踏み出しにより、  
最終的に着地点からの反力で重力を相殺できるので、  
静的安定な状態になります。

---

## サンプルコードの解説



---

## まとめ・次回予告

今回はvnoidパッケージの目標DCM計画について解説しました。

次回は歩行制御器stepping_controllerについて解説しようと思います。

次回： [010 - 歩行制御器](https://koomiy.github.io/posts/stepping_controller/)
