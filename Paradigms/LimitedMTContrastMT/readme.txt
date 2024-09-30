MTContrast
使用8 ，32 ，128 3*3 的刺激矩阵中在十字架位置上的刺激来作为sample 进行训练MTContrast； MT 在4个 stimuli中进行选择；保证其按单一stimulus的策略正确率为75% ；
85%的最难的阈值； 
Test 5 中5000 个trial 加入 probe trials （10%） ； test 6 中加入不同的delay period

同时训练MT 方法 6只；WSLSAnitbias方法3只；random 方法3只
使用超声波喇叭
delay period 设置为500ms；precuedelay 设置为 200ms；其中训练方法不变；去掉一开始的random；对应方法一直使用到最后；

使用 两个 S 刺激的模型；参数设置不变；但S 参数为 -1 0 1； 0.01 alpha

所有训练的一些trial type selection：
  在habitation 之后都使用MT 来建模
  S.trialPresentMode
  TrialContrast ,trialtype:
  1: random + WSLS antibias ; 
  2: MT + MT 
  0: limited + random ;  
  3: limited + MT ; 
  4 :limited + WSLS antibias
  5: random + random ; 
	         habitation0  habitation1 EarlyTraining0 EarlyTraining1 Retention Test1 Test2
MT                             1                  1                     2                 2                   2          3         3
Baseline antibias         1                  1                     1                 1                   4          4         4
Baseline random         1                  1                     5                 5                   0          0         0

3*3 MT		    0	         0                   3                  3                   3          4         4
3*3 random                 0                  0                    0                  0                    0        4          4
3*3 WSLSantibias        4                  4                   4                   4                    4        4          4

limitedMT                   0                  0                     3                 3                   3          3         3
limitedRandom           0                  0                     0                 0                   0          0         0
limitedAntibias            0                  0                     4                 4                   4          4         4     

使用参数为0.1 ； 0.9 ；0.1