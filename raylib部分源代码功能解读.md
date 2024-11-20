官网  
<https://www.raylib.com/>  
<https://github.com/raysan5/raylib>  
我根据自己的需求裁剪了多余功能后的代码：  
<https://gitee.com/xd15zhn/raylib>  
<https://github.com/xd15zhn/raylib>  

# 参考中文资料
- [raylib绘图库简介 - 知乎](https://zhuanlan.zhihu.com/p/458335134)
- <https://blog.csdn.net/qq_53530146/category_11925207.html>
- [个人笔记｜Raylib 的字体使用](https://www.bilibili.com/read/cv16839003)

# 源代码部分功能解读

## 延时函数与FPS显示
&emsp;&emsp;Windows下，raylib中的延时函数有3种方法，第一种是完全使用`Sleep()`函数，误差很大；第二种是完全阻塞CPU，精度很高；第三种是两种方法的折衷，前95%的时间使用`Sleep()`函数，最后剩下的时间阻塞CPU直到达到规定的延迟时间。我在第三种方法的基础上优化了原作者代码使该函数精度更高。
&emsp;&emsp;源代码中使用了`timeBeginPeriod`函数来提高`Sleep()`函数的精度，见官方文档 [timeBeginPeriod function (timeapi.h)](https://docs.microsoft.com/en-us/windows/win32/api/timeapi/nf-timeapi-timebeginperiod)。
&emsp;&emsp;帧周期计算的代码在`BeginDrawing()`和`EndDrawing()`两个函数中。一个帧循环过程大致如下
```c
while (!WindowShouldClose()) {
    User_Calcation_Code();  // 用户的逻辑和计算代码
    BeginDrawing();  // 这里统计用户代码耗时
    Drawing_Code();  // 绘图和渲染代码
    EndDrawing();  // 这里统计绘图代码耗时，并根据目标帧率延迟对应时间
}
```
&emsp;&emsp;一个帧周期分为3部分，用户代码耗时、绘图代码耗时和延迟时间。举个例子，设置目标帧率为50fps，则一个帧周期为20ms，设用户代码耗时2ms，绘图代码耗时1ms，则在`EndDrawing()`函数中调用延时函数延迟17ms。
&emsp;&emsp;最后在计算fps的函数中，原作者代码中使用30个数据进行滑动平均，因为我提高了延时函数的精度，所以删除了滑动平均的代码。我的计算量比较大，于是添加了计算CPU占用率的函数，也不使用滑动平均。每个帧周期都更新一次fps和cpu会看不清楚数字，所以我直接删除了从库中删除`DrawFPS()`函数，显示fps和cpu的功能直接在用户代码里写。每一帧还是要显示fps和cpu文字，但数值是半秒更新一次。

## 全屏显示
&emsp;&emsp;raylib的屏幕显示相关代码特别多，包括但不限于：设置是否全屏，全屏时可以设置在哪个显示器上全屏，非全屏时可以调整窗口尺寸，全屏和非全屏相互切换等。
&emsp;&emsp;我觉得全屏效果非常好，所以根据我的需求改了大量全屏相关的代码，比如删除了`InitGraph`函数中在所有显示器中找到分辨率最匹配的显示器的代码，改成了自己选择显示器。

## 相机
&emsp;&emsp;相机的代码太烂使我必须下决心大改源码，也是我第一个动手的地方。在源代码中，本来在`raylib.h`文件中已经在相机的结构体中定义了关键参数类型，核心源码`rcore.c`中也用的是这个结构体，但因为不同场景下需要相机的动作各不相同，于是在相机源码`rcamera.c`中扩充了原来的相机结构体，并写了几个常用的相机运动模式。我甚至觉得不是常用的而是各位代码贡献者们要用的。比如除了相机不能上下运动以外，还有一个轨道运动模式，我并不认为这是一个常用的功能。
&emsp;&emsp;我想删除多余的相机代码，但一个测试程序还是需要相机，纠结了很久想到了一个办法，只为`README.md`中的测试程序写一个最基本的默认相机程序，只能看到3D场景，其他源码全部删除，相机的核心结构体也精简了不少，然后不同的项目中再单独编写相机的运动代码。

# 修改与删减的内容
## 删减模块与功能
- `raudio`及所有音频功能
- 2D绘图
- 碰撞检测
- 删除了大量宏定义
- 骨架与姿态
- 动图

等。
## 增加功能
- CPU利用率

等。
## 重命名
- InitWindow

这个函数跟`<windows.h>`中的一个函数重复了，所以改成了跟2D图形库`EGE`和`easyx`相同的函数`InitGraph()`。
- Rectangle

同样这个结构体名跟`<windows.h>`重复了，改成了`rayRect`。
## 修改函数实现
- SetWindowMonitor

不知道它原本是做什么的，我给它改成在全屏时设置在哪个显示器上全屏。



