visual + audio + multi-modality 

using poisson events 

Protocols：
1. 固定的event rate；从poisson process 中生成；（training）
2. 固定的poisson rate，控制其event rate 在设定的范围内；同时trialtype不会完全按照设定出现，而是具有一定的随机性；同时sample period 具有一定的随机性200-1000ms（testing）


TSLoop event rate

该范式为 5 个 tasks  ,主要目的是看看是否会先选择简单的task进行训练，并最终通过阶梯的学习学会 light flashes task
（modality ，light iNtensity ）
1.light flashes(4 vs 20)(light intensity 10; ) 
2.light flashes(4 vs 20)(light intensity ; 20 )
3.audtory clickes(4 vs 20)
4.MultiModal (light + aud event rate)(light intensity 10; )(4 vs 20)
5.MultiModal (light + aud event rate)(light intensity 20; )(4 vs 20)

light panel (两个光强水平对应于 low light intensity 和high light intensity)
加入EL 的抑制学习；delayPeriod (200-500)
EarlylickDelay 100-300s  
对于EL trials 在 evaluation 阶段去掉 ；其中为training 阶段：2000 trials ；evaluation 阶段： 5 * 100 trials ;-> 2000/500;
 
使用 light intensity 2500K 

本范式的目标task 为light flashes的学习；只要完成了总共完成了10次eval 的 light flashes 的相关训练，即进入到test protocol，并使用该light flashes其他变量一致的多模态范式随机作为 test protocol

去掉 randomly delay period ；固定为200ms；

event rate total is 24ms ;

Sample period 设定为1000ms ；在test protocol 设置为1600 ；1200 ；1000； 800 ; 400 ms

pre delay period 为 100-200ms；bins 设定为 20/20 ms ;



