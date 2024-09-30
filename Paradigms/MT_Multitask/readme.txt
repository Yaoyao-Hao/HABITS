auditory frequency based
protocol：
1. Early shaping(学习 trial结构)
	1. free reward (randomly reward without corrsponding stimuli display)（decouple the stimuli and task structure learning）;center port trial began; pre-cue + 350ms minimum RT period in sample period timeout punishment with EL trials ( randomly 3s~7s)
Sample period is 1200ms; without delay period 
(8000 VS 32000Hz ; Light Intensity 1; protocol 0: 16000 Hz ;random reward)
2.Training
	1.add error trials

lick后会立即结束sample period

进入到Training阶段后，使用 machine teaching 算法；alignMax or iterative machine teaching

尝试：
1. 构建一个alignMax的算法，其中 逻辑回归的特征为 S0 ，S1～3 ，A1～3 ，R1～3  ，bias ，WSLS 12个特征；12个参数
	使用单步的梯度下降；可调的learning rate（对于reward unrewarding trial有不同的learning rate）和L1 正则参数和loss继承 率gamma；
	根据fidelity来设定阈值(难以实现)
	每一次的迭代的参数和超参数都保存到Trial.txt中,每个trial在之前的预测中是否成功和置信度也保存下来
	超参数和参数的存储放到一个单独的文件 ParamLR.txt 中，每次迭代都从中读取值，可以通过SD 远程功能随机进行更改对应的当前参数
	对于NoResponse的trial，去掉，记录的特征中都不包含No Response trial


关于model参数的解释与使用：
本质上分为两个模型；第一个为teacher model ；第二个为student model
在实际应用中，student model为小鼠；是隐变量模型，无法获得其真实的权重，并无法对其进行采样（待研究）
使用teacher model来拟合小鼠模型从而获得模拟的采样数据来进行example的选择
使用逻辑回归模型来进行拟合，其中更新的权重有两个部分；（按照momentum 更新算法思想）
第一个部分为 当前trial的loss产生的梯度；第二个部分为之前trial的loss的梯度的指数衰减值；
在更新过程中，对最终的梯度加入一个weight decay 项（L1：0.1）来防止过拟合（解耦 momentum 和weight decay）

关于参数更新：
在以外的研究中，发现小鼠在进行决策的时候，对待reward trial和unreward trial是不同的；具体表现为：
1、unreward/error trial后会将当前信息与历史信息解耦（第二部分loss 的梯度衰减加快）
2、对待reward trial 的学习率会比err trial的学习率大
将上述两项结合（改变 beta 值，在correct trial为0.9 ；error trial 为0.1）
3. learning rate （reward trial 0.1 ，error trial0.1或者0.05或者0.01）
4. eta learning rate（选择 trialtype过程中，模拟的小鼠对outcome的梯度）（reward ：1；err：0.1或者1或者0.8或者0.5）

 Aud freq + reverse + aud orientation + reverse + light orientation + reverse + transparent


给一只MT 小鼠 ，使用EMA 0 L1 0 learning rate 0.01 试一下
learrning rate 0.1 也试一下
	