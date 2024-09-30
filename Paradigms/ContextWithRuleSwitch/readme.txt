训练过程同attention 3ports
训练中 pre cue 设置为16 Hz VS 64 Hz
在test 中引入 pre cue 的contrast {16 ,20 ,25 ,32 ,40 ,50 ,64} ； 30% trials 会有contrast pre cue

low clicks rate（16 ；20；25 Hz） -> according to spatial auditory cue （Left VS right）
high clicks rate （40；50 ；64 Hz）-> auditory frequency （3KHz vs 12KHz）

对于 只出现 freq的trials ： 刺激来源于top蜂鸣器；
只出现 spatial 刺激的trials ： 刺激来源于左右；6KHz；
conflict trials ：刺激来源于左右，3KHz or 12KHz；

int modality = 0; // 0: spatial target ; 1: freq target ;2 :nodef
int ContextType = 0; // 0 is without-conflict  ;1 is conflict ;2 is coherent;3:nodef

context / modality     	output(left / right choice)
0/0			LeftCueSoundOutput / RightCueSoundOutput
0/1			LowSoundOutput / HighSoundOutput
1/0			LeftHighSoundOutput / RightLowSoundOutput
1/1			RightLowSoundOutput / LeftHighSoundOutput
2/0			LeftLowSoundOutput / RightHighSoundOutput
2/1			LeftLowSoundOutput / RightHighSoundOutput

sample period lick 会直接进入到feedback

block size 设置为10 -> 5