# Review
&nbsp;
> **Survey of imitation learning for robotic manipulation**(2019.12)
>
> B. Fang, S. Jia, D. Guo, M. Xu, S. Wen, and F. Sun,
>
> Journal : *International Journal of Intelligent Robotics and Applications*
>
> Categories:  ENGINEERING | ROBOTICS
>
>  *Web of Science* Core Collection: Emerging Sources Citation Index (ESCI)  
  
  
## 0. Abstract
    로봇 공학의 발달로 로봇의 응용은 산업 현장에서보다 지능적인 서비스 시나리오 점진적으로 진화했다. 복잡하고 불확실한 환경에서 로봇의 멀티 테스킹 작업을 위해 기존의 수동 코딩 방법은 번거로울뿐만 아니라 환경의 급격한 변화에도 적용하기 어렵다.  
  
  
     전문가 데모를 사용하여 초기부터 학습하는 것을 피하는 **모방학습**은 로봇의 조작을 위한 가장 효과적인 방법이 되었다. 이 논문은 로봇 조작의 모방 학습에 대한 설문 조사를 제공하고 미래의 연구 동향을 탐구하기 위한 것이다.  로봇 조작을 위한 모방학습 기술의 검토에는 데모 및 표현 및 학습 알고리즘 세가지 측면이 포함된다. 논문의 끝에서 우리는 미래의 연구 가능성 영역을 강조한다.




## 1. Introduction
  
​	 전통적으로 로봇의 조작 능력은 하드 코딩에 의해 획득된다. 다른 작업과 환경에 맞게 조정하려면 다른 프로그램이 필요하다. 로봇이 동적으로 변화하는 이벤트를 처리하기는 어렵다. 이는 로봇의 실행 가능성과 효율성에 직접 영향을 미치므로 로봇의 조작 능력을 향상시키기 위해서는보다 지능적인 방법이 필요하다.
  
​	인공 지능의 빠른 개발과 함께 머신 러닝 방법은 로봇이 동적 환경에서 조작 기술을 배우고 습득하는 데 사용되며, 이는 전통적인 프로그래밍 방법의 결함을 보완한다. 오늘날 데모, 견습 학습으로도 알려져있는 **모방 학습**은 로봇이 기술을 습득하는 가장 효과적인 방법이되었다. 기존의 방법과 비교하여 모방 학습 알고리즘은 특정 장면과 작업을 위해 하드 코딩을 피한다. 
  
​	**모방 학습** 메커니즘의 장점은 다음과 같이 요약된다.
>  
>  1.  **적응력향상(Enhancing Adaptability)**
>  
>    개별의 로봇이 다른 것의 움직임을 관찰하여 모방 할 수 있는 능력이 있다면 새로운 환경에서 유용한 행동을 빠르게 배우고 적응할 수 있다. 
>  
>  2. **커뮤니케이션 효율성 향상(Improving Communication Efficiency)**
>  
>    모방은 비언어적인 의사소통으로써 개체가 다른 유형이나 하드웨어의 다른 개체에게 배울 수 있는 효과적인 수단으로 제공된다. 각각의 행동하는 동안 많은 양의 중요한 정보를 전송하므로 커뮤니케이션도 효율적이다.
>
>  3.  **학습 효율성 향상(Improving Learning Efficiency)**
>  
>     모방 학습의 가장 큰 이점은 학습 과정에서의 높은 효율성이다. 개체가 새로운 행동을 습득하면 개체들에게 빠르게 확산된다. 
>     모방은 모든 개체의 학습 능력을 결합하여 최선의 행동을 빠르게 확산시킴으로써 전체 그룹의 생존력과 적응력을 향상시킨다. 
>  
>  4.  **다른 학습 메커니즘과 호환(Compatible with Other Learning Mechanisms)**
>  
>    모방 학습은 학습의 속도와 정확성을 향상시킬 수있는 강화 학습처럼 다른 기계 학습과 결합할 수 있다.
>  


​	이러한 장점을 활용하여 최근 몇 년 동안 모방 학습이 빠르게 발전하여 로봇 학습 분야에서 뜨거운 연구 주제가됐다. 로봇 시스템의 모방 학습 과정은 일반적으로 아래 그림과 같이 **Demonstration**, **Representation**,  **Imitation Learning Algorithm**의 세 부분으로 구성되며, 다음 섹션에서 세 부분의 현재 연구 상태를 분석한다.

> *Demonstration*은 모방학습과정의 기본으로써 전문가의 조작정보를 획득하는 과정을 말합니다.
>
> *Representation*은 앞선 시연 과정에서 획득한 데이터를 로봇에 적용될 수 있도록 특성화시키는 것입니다(정책의 추상화 단계).
>
> *Imitation Learning Algorithm*은 앞선 리프레젠테이션 과정 이후 러닝을 하여 최종적으로 행동을 "마스터" 하게 한다. 여기서 행동을 "마스터"한다. 이 의미는 모방학습의 궁극적인 목표와 동일하며, "로봇이 행동을 그대로 재현하고 다른 미지의 환경에서 행동을 일반화할 수 있는 것"을 말한다.



## 2. Imitation Learning for Robotic Manipulation



### 2.1 Demonstration

​	조작법의 습득은 모방학습 과정의 기본인 관찰(*Observation*)을 통해서 전문가의 조작 정보를 획득하여 흉내내는 과정을 말한다. 현재 모방학습의 시연 방법은 크게 **Indirect Demonstration**와 **Direct Demonstration**의 두 가지 범주로 나눈다.



#### 	2.1.1 간접 시연(Indirect Demonstration)

​	**간접 시연**는 로봇과의 접촉이 필요하지 않지만, 그 대신에 시연을 위해 별도의 환경을 구축하여 교육과정에서 조작 정보를 수집한다. 간접 교시는 시각 시스템이나 웨어러블 장치를 사용하여, 아래 그림과 같이 인간의 행동 정보를 인식하여 로봇이 의인화된 조작을 할 수 있도록 생성한다.

의인화


> ​	(1) *Visual Indirect Teaching*은 기계학습을 통해 전문가의 이미지를 관측하는 것이며, 이 방법은 높은 학습 속도로 핫한 연구 주제가 되었다. 하지만 이러한 교시 샘플은 로봇 조작에 결정적인 촉각 정보와 같은 정보들이 부족하다.
>
> ​	(2) *Wearable Indirect Teaching*은 웨어러블 센서를 통해 샘플을 수집하여 정확하고 다양한 정보를 얻는다. 하지만 간접 교시는 로봇과 분리되어 있기 때문에 로봇 자체의 작동 특성을 고려하지 않으므로 데모의 품질을 보장할 수 없다.



#### 	2.1.2 직접 시연(Direct Demonstration)

​	**직접 시연**은 로봇에서 직접 교시 샘플을 얻으므로 프로세스가 빠르고 간단하며,  동작이 더 정확하다. 직접 교시는 아래 그림 같이 *Kinesthetic Teaching*과 *Teleoperation Teaching*으로 나눌 수 있다.

 로봇의 동작이 더 정확합니다. 직접 교육은 그림 3과 같이 **Kinesthetic Teaching** (Amir and Matteo 2018)과 **Teleoperation Teaching** (Zhang et.al. 2018)으로 나눌 수 있습니다.  


> ​	(1) *Kinesthetic Teaching*은 작업자가 로봇과 직접 접촉하고 안내하여 특정 작업을 완료하고 로봇이 자체적으로 정보를 수집하는 것을 의미한다. 이 교시 방법은 로봇과 인체 간의 다른 기구학적 파라미터를 고려할 필요가 없고 수집된 훈련 데이터의 노이즈가 비교적 낮은 수치를 가지며, 직관적으로 제어 프로세스를 반영한다. 하지만 이러한 상호작용에 적합한 로봇은 수동 제어가 가능해야 하고 직접 접촉을 해야 한다. 그렇기 때문에 다중 자유도 로봇 팔이나 듀얼 암 로봇과 같은 로봇에 적합하지 않다.
>
> ​	(2) *Teleoperation Teaching* 은 조이스틱, 촉각 센서, 컨트롤 패널, 적외선 센서, 웨어러블 장치 이나 다른 원격 교시 디바이스로부터 수행한다. 데모는 교시 과정에서 로봇 자체를 구속하지 안고 전문가와 로봇 간에 직접적인 접촉이 없어 안정성을 보장한다. 그 결과 원격 시연은 높은 안전성과 넓은 적용 범위으로 인한 이점으로 높은 수준의 전문가 샘플을 얻을 수 있다. 이를 위해 *Li Feifei's* 팀은 시연을 위해 RoboTurk 네트워크 플랫폼을 구축했다. 시연자는 휴대폰이나 마우스를 사용하여 로봇 암을 원격으로 자유롭게 조작할 수 있고 아래 그림과 같이 편리하게 많은 양의 데이터 셋을 얻을 수 있다. 하지만 현재 대부분의 원격 조작은 자세 또는 궤적을 기반으로 한 교시이다. 실제 작동하는 힘의 정보가 부족하고 정교한 작업을 수행하기 어렵다.



### 2.2 Representation(Policy Abstractions)

 	전문가가 조작을 완료하고 나서 샘플은 환경, 조작 객체(*Operational Object*) 등과 같은 많은 특징을 포함하고 있다. 하지만 대부분 시연에서 조작 환경은 로봇과 어울리지 않는 관련 없거나 중복된 많은 양의 특징을 포함하여 복잡하다. 이것은 이후에 진행하는 모방하는 과정을 방해한다. 따라서 시연을 로봇에 적용 및 인식될 수 있도록 효율적으로 특성화시키는 것이 중요하다. 모방학습에서 조작 시연은 세 가지 유형  **Symbolic Characterization**, **Trajectory Characterization**, **Motion-state Spatial Characterization**으로 나눌 수 있다.



#### 2.2.1 상징적 특성화(Symbolic Characterization)	


​	**Symbolic Characterization**에서 학습을 통해 일련의 옵션(*Option*)을 생성하고 일정 시간 동안 취하는 행동에 대한 정책으로 정의한 것이다.목록에서 다른 옵션(*Option*)을 선택하는 것으로 다른 작업을 수행한다. 복잡한 작업의 경우, 단일 행동(*Action*)으로 수행하기 어렵기 때문에, 상징적 표현(*Symbolic Representation*)은 옵션(*Option*)에서 단순한 행동의 집합을 정렬함으로써 복잡한 작업으로 특성화시킨다. 상징적 표현은 모방 학습에서 로봇의 높은 수준의 행동 표현이며, 다음과 같은 장점을 가진다.

> + Symbolic Characterization의 장점
>
> ​	(1)  The same basic action can be reused(재사용) to accomplish different tasks. 
>
> ​	(2)  Convenient to replace the actions in the option’s action sequence to adjust the relevant motion plan.

​	상징적 표현을 이용한 모방 학습은 multi-modal robot에서 편리한 방법이며, 복잡한 multi-step task의 학습 문제를 해결하기 위한 실용적인 해결책이다. 하지만 복잡하고 정교한 변형을 해야하는 경우, 상징적 특성화를 통해 모션을 제안하거나 정확하게 분할하기 어렵다.



#### 		2.2.2 궤적 특성화(Trajectory Characterization)

​	**Trajectory Characterization**는 궤적(*Trajectory*)과 작업의 조건(*Task-Related  Condition*)을 맵핑하는 것으로, 앞선 상징적 특성화와 비교하여 낮은 수준의 표현이다. 여기서 작업의 조건(*Task-Related  Condition*)은 예를 들어, 그랩핑 작업에서 그리퍼의 초기 위치,  물체의 목표 위치와 같은 시스템의 상태를 말한다. 궤적(*Trajectory*)은 시계열 시스템의 입력과 상태로 추상화시킬 수 있다.  상징적 표현(symbolic representation)에서 행동(*Action*)의 집합의 각 행동(*Action*)을 궤적 특성화로부터 특성화 시킬 수 있다.

> + Trajectory Characterization을 적용한 논문
>
>   (1) Trajectory characterization is like the **dynamic motion primitive method** in behavioral cloning, which introduces an additional forcing term in the critically damped spring system to normalize its characterization of complex individual actions(*Pastor et al.* 2009).
>
>   (2) Another example is the **probabilistic motion primitive method** used in *Dermy’s* paper, which describes the trajectory as a motion primitive distributed by probability (2017).
>
>   (3)  In practical applications, *Yang’s* paper extracted the impedance of the human body and applied it to the robot, which extended the **dynamic motion primitive method** (2017).

​	하지만 궤적 특성화 방법은 교시 샘플이 가능한 많은 동적 요소들이 포함되어 있어야 하므로 이미지나 터치와 같은 비 동적 특징만 포함하는 샘플에서 수행하기 어렵다.



#### 	2.2.3 행동-상태 공간 특성화(Action-State Spatial Characterization)

​	**Action-State Space**는 앞선 다른 두 가지의 특성화와는 다른 특징이 있다. 사전에 행동(*action*)들과 옵션(*option*)들을 생성하고 특성 상태(*state*)가 발생하는 경우, 일련의 행동-상태 판단(*action-state decision*)을 생성한다. 상응하는 제어 행동(*action*)은 작업 조건(*Condition*)과 제어할 수 있는 상태(*state*) 사이 맵핑 관계로 정해진다.

>Action-state space characterization을 적용 논문
>
>​	(1)  The method of motion-state space representation is used in **dynamic system stability observation method** of behavioral cloning.
>
>​	(2) This method obtains a **nonlinear autonomous dynamic system** capable of generating action-state decision (*Khansari-Zadeh and Billard* 2011).
>
>​	(3)  Action-state space characterization also includes the inverse reinforcement learning method used in *Faha et al.’s* and *Piot et al.’s* paper, which assumes the optimality of expert teaching and learns the reward function and uses the reward function to generate an action-state decision(2016/2018).

​	하지만 행동-상태 공간 특성화는 순간적인 행동에 대한 결정(*decision*)을 특징으로 하기 때문에, 긴 기간동안의 표현(*Representation*) 과정에서 특성화 에러를 쉽게 축적한다. 

<img src="C:\Users\Joseonghyeon\AppData\Roaming\Typora\typora-user-images\image-20200407141110574.png" alt="image-20200407141110574" style="zoom:67%;" />

​	Demonstration은 Dynamic, Vision, Touch와 같은 다양한 요소가 포함된다.

> + single operational information characterization 관련 논문
>
> ​	(1)  *Yang* indicates the **dynamic characteristics of the joint angle, joint velocity and end pose of the robot arm** are generated by teaching, and the operation is characterized by the trajectory characterization (2018). 
>
> ​	(2)  The humanoid robot made by *Hwang et al* simulates human motion through visual observation, creating a three-dimensional image sequence using a stereoscopic vision capture system, and the extracted **visual features** are then used to estimate the trajectory of the teacher (2016). 

​	추가적인 시각적 특징과 촉각 정보, 동역학도 매우 중요하다. 촉각 신호의 도입은 최근 핫한 연구 주제가 되었다. 지난 몇년 동안  K-Nearest Neighbor, Support Vector Machine, Gaussian Process, Nonparametric Bayesian Learning 등 많은 머신러닝 방법을 통해 촉각 매트릭스을 성공적으로 식별했다. 

>+ multi-modal operational information characterization관련 논문
>
>(1)  The tactile features are firstly introduced in *Liu et al.*, which improves the success rate of robotic cap opening operations. At present, most of the research still focused on the operational characterization of **dynamics and visual features** (2019). 

​	현재 대부분의 연구는 역학과 시각적 특징의 특성에 중점을 두고 있다. 하지만 촉각을 포함한 복합정보 특성화는 더 나은 특성화 결과를 얻을 수 있도록 확장시킬 수 있다. 따라서 Multi-modal operational information characterization에 대한 연구는 모방 학습에 큰 의미가 있다.




### 2.3   Imitation learning algorithm 

​	로봇 모방 학습의 전체 과정에서 시연은 많은 특징을 포함하는 교시 샘플을 제공하며,  operation characterization는 로봇이 인식할 수 있는 유효한 형식으로 교시 샘플의 특징을 특성화한다. 모방학습의 궁극적인 목표는 로봇이 행동을 재현하고 다른 미지의 환경에서 행동을 일반화해야 하며, 이를 "Master" 행동이라고 한다.  이러한 "Master"행동을 위해 로봇이 교시정보를 사용하는 프로세스를 **Operation Imitation**이라 할수 있다. 이러한 Operation Imitation의 방법에는 크게 *Behavioral Cloning*, *Inverse Reinforcement Learning*, *Adversarial Imitation Learning* 의 세 가지 범주로 나눈다.

#### 2.3.1 행동 복제(Behavioral Cloning)

 	**행동 복제** 방법은 지도학습과 유사하며, 교시 정보를 사용하여 궤적(*Trajectory*)과 행동(*Action*)으로 상태(*status*) 및 작업 조건(*Condition*)을 직접 맵핑한다. 모델에 의존 여부에 따라 (1) 모델 기반 행동 복제와 (2) 비 모델 기반의 행동 복제로 나눌 수 있다.  또한, 조작의 특징화에 따라 (1) 궤적 특성화 행동 복제와 (2) 상태-행동 공간 특성화 행동복제 (3)  상징적 표현 행위 복제로 나눌 수 있다.  두 가지 분류에 따라 자유롭게 결합되어 많은 유형의 행동 복제 방법이 있다.

> Behavioral Cloning 관련 연구 동향
>
> ​	(1)  Examples are the **dynamic motion primitives** in *Ijspeert et al.*(2013) and the **mixed motion primitive methods** used in *Gams et al*.(2014) and *Amor et al.* (2014), such methods can generate continuous, smooth and generalizable representations in terms of trajectories. 
>
> ​	(2)  In *Andrew* (2015), picture information is taken as the state, and steering wheel angle is taken as the action, thus the state-action space decision is learned. 
>
> ​	(3)  Therefore, in *Finn et al.* (2016), a dynamic model of robot with high-degree of freedom is established using a **deep network**.

​	행동복제는 샘플의 수가 제한되는 경우 적합하지 않다. 



#### 	2.3.2 역강화학습(Inverse Reinforcement Learning)

​	**Inverse Reinforcement Learning**은 전문가의 교시정보를 최적으로 가정함으로써 전문가의 의도를 나타내는 보상함수를 추정하고 추정된 보상함수를 기반으로 최종적으로 제어하기 위해 강화학습을 사용한다. 역강화학습의 경우 교시 샘플이 충분하지 않더라도 보상함수를 역으로 추정할 수 있어 Generalized Strategy를 도출할 수 있다.  역강화학습은 행동복제와 동일하게 분류된다.

>+ Inverse Reinforcement Learning 관련 연구 동향
>
>​	(1)  *Ratlif et al.* find a reward function that can make the optimal strategy the most different from other strategies (2006). Based on this, it can be generalized to a reward function that can be used for nonlinearity(*Zucker et al.* 2011).
>
>​	(3)  *Ziebart et al.* used the probabilistic model to propose the inverse reinforcement learning of **the maximum entropy model**, which overcame the random deviation of the reward function caused by the expert teaching preference (2008).
>
>​	(4)  In terms of application, *Finn et al*. obtained a **nonlinear reward function** through inverse reinforcement learning, which can guide the robotic arm to complete complex housework tasks (2017).

​	역 강화학습은 전문가의 교시 정보의 최적성에 기반하여 보상 함수를 복원하여 교시환경과 매우 다른 환경에서 덜 효과적이다.



#### 	2.3.3 Adversarial Imitation Learning

​	행동 복제와 역강화학습은 오로지 전문가의 교시 정보를 바탕으로 학습하지만, 전문가의 교시 정보와 상호작용을 하지 않는다. 하지만 **Adversarial Imitation Learning**의 생성은 Generative Adversarial Network(GAN)과 Imitation Learning이 결합된 방법이다.  여기서 GAN은 2014년에 발표되었으며, Natural Language Processing 분야에서 많은 성과를 보여주었다. GAN은 생성자(Generator)와 식별자(Discriminator)가 있어 서로 대립하여 서로의 성능을 점차 개선해 나가는 개념이다. 이를 착안하여, 모방학습에 적용하면 위의 그림과  같이 생성된 데이터(*Generator*)와 초평면(*Discriminator*)로 나누고 서로 반복적인 모의 훈련을 수행하여, 두 데이터 사이를 가능한 가깝게 분포할 수 있도록 한다. 이는 결국 생성된 데이터(*Generator*)와  전문가의 데이터 패턴과 유사하게 된다.

> + Adversarial Imitation Learning 확장 관련 논문
>
> ​	(1)  *Baram et al.* proposed a method based on forward model to make the stochastic strategy completely differentiable to generate imitation learning (2017).
>
> ​	(2)  *Henderson et al.* proposed an imitation learning method for the optional framework of hierarchical strategy (2018). 

​	AIL에서는 생성된 궤적과 전문가의 교시 궤적을 경쟁시켜, 생성된 궤적이 실현 가능성과 목표에 도달 가능한지를 시뮬레이션하는 것이 필수적이다. 그렇기 때문에 모델 정보가 충분하지 않은 시스템은 AIL을 적용하기 어렵다.



### 3. Discussion and Conclusions 

​	요약하면, 모방학습은 로봇 조작에서 중요한 핵심 기술이 되었다. 그러나 이 분야의 현재 연구작업은 다음과 같은 문제에 직면해 있다.

#### 3.1 Demonstration 

 	 Although great progress has been made in teaching via wearable devices, most of Teleoperation teachings only consider the position and posture, and most of them are the mechanical arms of the end jaws, lacking information about the overall operation of the hand-arm system. For a multi-degree-of-freedom humanoid manipulator, it is necessary to consider the operational configuration, position, attitude, and the dexterous hand’s operating force, that is, the tactile teaching. But how to integrate multi-modal teaching methods to achieve high-quality teaching samples remains a challenging issue.  

#### 3.2 Representation

​	현재 대부분의 연구는  Trajectory이나 Visual Representation에 중점을 둔다. 하지만  Visual과 Tactile Representations은  모방학습에 더 많은 정보를 제공할 수 있다. 로봇 조작에서 Visual과 Tactile 정보 간의 상관 관계를 이용하는 방법이나 복합정보의 특성화를 배우는 방법은 실제 응용에서 매우 중요한 문제이다. 이 분야 연구는   이제 시작되었다. 그리고 이는  Operational Characterization의 초석뿐만아니라 미래 복합 교시 정보 특성화를 위한 중요한 방향이다.

#### 3.3 Learning

​	  The existing imitative operation learning has a low utilization rate of teaching samples, and cannot achieve efficient strategy learning. The operation of the imitation learning algorithm is more sensible for **multi-modal characteristics, operational space locality, and small samples** which pose a great challenge to the generalization of imitation operations How to design an efficient **robotic imitation learning framework** is still at the forefront of robot learning. In general, the **multi-modal imitation learning** of robotic operation technology provides a more **efficient and high-quality way** for the robot to better grasp the operational skills, which is of great significance for improving the operational intelligence of robots. There are still many challenging academic problems in this field, and it is **necessary to carry out in-depth exploration and analysis** from the perspectives of signal processing, machine learning, and robot operation theory 
