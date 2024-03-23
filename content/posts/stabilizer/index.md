---
title: "013 - 安定化制御"
date: 2024-03-23
categories: ["vnoidの解説"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、  
安定化制御器(Stabilizer)について解説します。

---

## 安定化制御の概要

これまでの記事で、人型ロボットを線形倒立振り子として目標運動を計画しましたが、  
残念ながら完璧にその通りに動作することはありません。  
計画運動からずれて不安定化してしまったときに安定化制御を行い、  
転倒を回避しなければなりません。

安定化制御では、ロボットの回転運動を抑えるための転倒回復モーメントを実現させます。  
そのために、目標着地位置を計画から修正します。  
これは、例えばあなたが電車に乗っていて、  
急ブレーキがかかると後ろに倒れそうになりますが、  
倒れる方向に足を踏み出して踏ん張る現象と同じです。

本記事では、その着地位置修正がどのような仕組みで行われているかを説明します。

---

## サンプルコードの解説

-   **安定化制御全体の流れ**

```cpp {linenos=inline}
void Stabilizer::Update(const Timer& timer, const Param& param, const Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){
	const Step& stb0 = footstep_buffer.steps[0];
    const Step& stb1 = footstep_buffer.steps[1];
    int sup =  stb0.side;
    int swg = !stb0.side;

	// calc zmp from forces
    CalcZmp(param, centroid, foot);

	// error between desired and actual base link orientation
	Vector3 theta = base.angle  - base.angle_ref;
	Vector3 omega = base.angvel - base.angvel_ref;

	// calc base link tilt
	CalcBaseTilt(timer, param, base, theta, omega);
	
	// calc dcm and zmp 
	CalcDcmDynamics(timer, param, footstep_buffer, base, theta, omega, centroid);

	// calc desired force applied to CoM
	centroid.force_ref  = param.total_mass*(centroid.com_acc_ref + Vector3(0.0, 0.0, param.gravity));
	centroid.moment_ref = Vector3(0.0, 0.0, 0.0);

	// calculate desired forces from desired zmp
	CalcForceDistribution(param, centroid, foot);

	for(int i = 0; i < 2; i++){
		// ground reaction force control
		if( foot[i].contact ){
			for(int j = 0; j < 3; j++){
				dpos[i][j] += (-force_ctrl_damping*dpos[i][j] + force_ctrl_gain*(foot[i].force_ref[j] - foot[i].force[j]))*timer.dt;
				dpos[i][j] = std::min(std::max(-force_ctrl_limit, dpos[i][j]), force_ctrl_limit);

				drot[i][j] += (-moment_ctrl_damping*drot[i][j] + moment_ctrl_gain*(foot[i].moment_ref[j] - foot[i].moment[j]))*timer.dt;
				drot[i][j] = std::min(std::max(-moment_ctrl_limit, drot[i][j]), moment_ctrl_limit);
			}

			// feedback to desired foot pose
			foot[i].pos_ref   += -dpos[i];
			foot[i].angle_ref += -drot[i];
            foot[i].ori_ref = FromRollPitchYaw(foot[i].angle_ref);
		}
	}

}
```



---

-   **CalcZmp**

```cpp {linenos=inline}

```



---

-   **CalcBaseTilt**

```cpp {linenos=inline}

```



---

-   **CalcDcmDynamics**

```cpp {linenos=inline}

```



---

-   **CalcForceDistribution**

```cpp {linenos=inline}

```



---

## まとめ・次回予告

今回はvnoidパッケージの安定化制御  
(Stabilizer) について解説しました。

ひとまずvnoidの標準機能に関する説明は以上となります！