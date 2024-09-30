5*5 SGM
4 种难度；对应于4种sub tasks
ALP-based ；
softmax 参数微调
从最开始就引入ALP 方法；
与baseline random 进行对比；同时使用 limited MTContrast的baseline 作为一个基础的baseline 对比
antibias 算法与 random baseline 保持一致；WSLS antibias 或者 random；

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
  6: ALP      + random;
  7:ALP      +  WSLS antibias;
	         habitation0  habitation1 EarlyTraining0 EarlyTraining1 Retention Test1 Test2
MT                             1                  1                     2                 2                   2          3         3

Baseline antibias         1                  1                     1                 1                   4          4         4
Baseline random         1                  1                     5                 5                   0          0         0

ALP                              1                 1                     7                 7                   4           4         4        ***** 

limitedMT                   0                  0                     3                 3                   3          3         3
limitedRandom           0                  0                     0                 0                   0          0         0
limitedAntibias            0                  0                     4                 4                   4          4         4     

                 