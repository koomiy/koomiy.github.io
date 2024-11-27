---
title: "010 - プレチャレンジ2023"
date: 2023-12-14
categories: ["過去のイベント結果 & レビュー"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

2023年11月11日に本イベント初の試みとなるプレチャレンジが開催されました。  
その結果とレビューをしたいと思います。

---

## 全体を通して

主催者の田﨑先生と、choreonoidの中岡さんからのご意見やご質問が多かった印象です。  
お二人とも今後さらに良くしていけるようにと、アドバイスをしてくださいました。

全体を通してアスレチックとダンス種目の参加者が多かったですが、  
短距離走種目に参加されるチームも１チーム見られました。

また、ディスカッションの時間では、  
メイン大会のオンラインとリアルのハイブリッド開催に関する案や、  
ダンス種目のレギュレーションに関する案が共有されました。  

ハイブリッド開催については今後検討とのことでした。

ダンスについて、課題曲は今大会は指定曲のみという方針が決定されました。  
評価は技術（うまく制御できているか）や、芸術（いくつか尺度を用意して投票）  
の２点から行うという方針が決定されました。  
詳しい評価方法については今後決定されるそうです。

---

## アスレチック種目について

本年度よりアスレチックコースが改装されました。  
初心者向けのコースと、玄人向けのコースに分かれておりますが、  
玄人向けのコースは難度が高く、手を出せているチームは本イベントでは確認できませんでした。  
プレチャレンジでは低難度コースについて、２チームの進捗を確認することができました。

１チーム目は個人で参加されているNWKさんです。  
A*アルゴリズムを使った移動経路生成法を取り入れており、  
自律動作での完走を目指しているそうです。  
現状、低難度コースの階段エリアの手前まではvnoidを用いて踏破できているそうです。  
階段エリアの自律歩行はまだ試せていないそうで、今後の課題とされていました。

２チーム目は私たち Team MA1です。  
ジョイコンを使った操縦動作での完走を目指しております。  
進捗状況はNWKさんとまったく同じですが、  
階段エリアの踏破のために、深度カメラによる段差検知を実装しました。  
今後は段差を考慮した安定化歩行制御を行う予定です。

---

## ダンス種目について

本年度より新設された種目です。
ダンス種目参加者にはFriendsのモデルが提供されるという好条件でのキックオフとなりました。  
さらに優勝チームには実機での実演チャンスが贈呈されるかも！？  
ということで皆さん注目のダンス種目ですが、  
プレチャレンジでは３チームの進捗を確かめることができました。

１チーム目は愛工大と岐阜大よりご参加のteketekeです。  
認知科学の観点から踊りを研究されており、  
その知見を活かして手の所作を主体としたダンスモーションを作るとのことでした。  
人の美的感覚にマッチした手の動きに注目です。

２チーム目は早稲田大の橋本先生の研究室よりご参加のチームです。  
モーションキャプチャーを使ってダンスモーションを生成し、  
それを人型ロボットに適用するという方針とのことでした。  
現状ダンス種目に向けた開発のみですが、  
今後はアスレチック種目の参加も予定されているとのことでした。

３チーム目は主催者の田﨑先生です。  
以前の技術講習会イベントにてChoreonoidの中岡さんが講演してくださった内容をベースに  
ダンスモーションを制作されておりました。
本イベントでは課題曲のイントロ２０秒間ほどのダンスモーションを披露してくださいました。  
非常に完成度が高く、ヌルヌルとダンスしていて驚くばかりでした。  
また、ロボットの運動学上の理由からどうしても実現しにくいポーズがあり、  
そのポーズを美しく補う方法を考える必要があるという課題を提示していただきました。

---

## 短距離走種目について

学生&社会人の混合チームがプレチャレに参加してくださいました。  
速く走る学習モデルを構築することを目指しているとのことでした。  
学習を用いるチームは本チームのみでしたので、具体的な手法が気になりますね。  
メイン大会での発表が楽しみなチームの一つです。

---

## まとめ・次回予告

今回はプレチャレンジ2023の結果とそのレビューを記事にしました。

同じ競技に様々なアプローチで取り組むチームの発表を聞いていて、  
大変良い刺激になりましたし、参考になる情報が盛りだくさんのイベントでした。

次回から再びvnoidライブラリの解説に戻ります。

次回： [011 - 歩行制御器](https://koomiy.github.io/posts/stepping_controller/)