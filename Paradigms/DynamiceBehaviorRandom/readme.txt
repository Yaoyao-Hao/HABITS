使用两种规则；
第一个为声音的频率；第二个为声音的方向； 会同时出现
不使用cue 来提示 小鼠 任务所使用的规则；而是使用prior ；
使用MT 算法来控制小鼠的规则转换；来帮助小鼠更加快速的进行规则转换；（使用不同的Weight goal）
（备选的刺激可以为两个都出现也可以为只出现一个）
（参数可以为 0.1 0.9 0.1或者 0.01 0 0 ） （做模拟测试） (加正则项 也许是很重要的； 相当于一个decay)
在模型中 把WSLS参数换为 Lose-shfit spatial or aud freq  ； win-stay 。。。 相当于把WSLS分为多个参数

可以设置多个block 对应于多个 target weights  
1. spatial
2. aud freq
3. bias right
4. bias left
5. reversal learning 
6. history dependence

注意： 最好不要做 prior 来选择
order ： spatial {1 ,0}-> aud freq & spatial  {1 ,1}-> reverse spatial {-1 ,1} -> reverse freq {-1 ,-1} 
也可以尝试一下 做一个 学习路径的问题：
{0,0} -> {1 ,1} -> {-1 ,-1}
{0 ,0} -> {0 ,1} -> {1 ,1} -> {1 ,0} -> {0 ,0} -> {0 ,-1} -> {-1 ,-1}

TODO 讨论 作为 下一批小鼠的训练

Protocol 0： Habitation 1
Protocol 1： Habitation 2
Protocol 2： Early training {10 ，0}
Protocol 3： Early training {10 ，0}
Protocol 4：retention {10 ，0}
Protocol 5：rule switch + retention{0 ，10}
Protocol 6： rule switch + retention{-10 ，0}
Protocol 7 ：{0 ，-10}
Protocol 4 - 5 - 6 - 7 循环