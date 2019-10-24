# matlabAutoDriving  
matlab auto driving tool examples  
1.1.testMonocularCameraandSemanticSegmentation1用于语法神经网络分割当前图片的地面区域，注意需要segnetVGG16CamVid.mat支持。下载神经网络和具体教程auto driving tool的文档  
1.2.testParkPathPlanning1.m用于自动停车的演示，基本所有auto driving tool关于控制，规划的方法都包含。具体教程见auto driving tool的文档   
1.3.mainTestNAV1是navigation工具库get start的四元数例子。具体教程见navigation工具库的文档   
1.4.mainTestNAV2是navigation工具库get start的IMU模拟建模例子。具体教程见navigation工具库的文档     
1.5.mainTestNAV3是navigation工具库get start的小车估计姿态和位置建模例子。具体教程见navigation工具库的文档  
1.6, mainTestNAV4是navigation工具库get start的路径估计例子。具体教程见navigation工具库的文档  
1.7.注意有些基于LIDAR的进行建立地图的例子，在GITHUB\RacerCar2019_ROS\ROS代码matlab1中，请看看。将会出现在  mainTestNAV5
1.8.NAV  navigation工具库的多车的车道转换和十字路口规划，还还没有实现  
      

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
2.1 mainTestDS1.m例子程序用于学习用代码实现DrivingScenario  
2.2 mainTestDS2.m例子程序用于学习用app实现DrivingScenario  
2.2 myScenario3.slx+myScenario3.mat 运行成功的ACC+Driving Simulation例子

注意
1.myScenario3运行前必须先运行myACCSetUp.m，初始化各种变量。而myScenario3.mat是的Driving Simulation APP的场景定义文件
2.myScenario3运行保证busActor和BUSACTORSACTORS的存在，可以直接重命名  
3.myScenario3运行是MPC工具库中ACCTestBenchExample的例子，注意细节上要改了很多很多，对照着ACCTestBenchExample模块设定参数。否则，会出很多错误。  
4.道路半径R设为很大，就等于直线道路
5. 修改 myACCSetUp3.m设定适合的初始参数保证ACC顺利运行  
6. generateMyScenario3.m给予程序生成scenario用APP保存为myScenario4.MAT文件  
具体见https://www.mathworks.com/help/releases/R2019b/driving/ug/create-driving-scenario-variations-programmatically.html  
2.建立使用MPC的ACC模型myACCSetUp3MPC.m和myScenario3MPC.slx  
