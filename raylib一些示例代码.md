作者：[找不到服务器](https://github.com/xd15zhn)

# 几种相机视角
&emsp;&emsp;相机视角放在项目代码里而不是raylib的库代码里，因为每个项目对相机视角的要求都太不一样。常用的几种视角的代码在下面给出。
## 头文件
几种相机视角共用一个头文件。
```c
#ifndef UTILS_H
#define UTILS_H
#include "raylib.h"

#if defined(__cplusplus)
extern "C" {
#endif

void Init_Camera(Camera *camera);
void Update_Camera(Camera *camera);

#if defined(__cplusplus)
}
#endif

#endif // UTILS_H
```
## 3D预览图视角
鼠标拖动，滚轮缩放。只能沿着中心点。
```cpp
/*3D预览图视角*/
#include <math.h>
#include "camera.h"

#define SENSITIVITY                      0.01f
#define CameraMoveExponential            0.9f
#define VAL_LIMIT(x, min, max)           (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))

float yaw, pit, dist;

void Init_Camera(Camera *camera)
{
    Vector2 vec;
    camera->position = (Vector3){ 0.0f, -20.0f, 20.0f };
    camera->target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera->up = (Vector3){ 0.0f, 0.0f, 1.0f };
    camera->fovy = 45.0f;
    vec.x = camera->target.x - camera->position.x;
    vec.y = camera->target.y - camera->position.y;
    yaw = atan2f(vec.y, vec.x);
    vec.x = sqrtf(vec.x*vec.x + vec.y*vec.y);
    vec.y = camera->target.z - camera->position.z;
    pit = atan2f(vec.y, vec.x);
    dist = sqrtf(vec.x*vec.x + vec.y*vec.y);
}

void Update_Camera(Camera *camera)
{
    static Vector2 mousePosPre;
    Vector2 mousePosNew, mousePosDelta;
    float mouseWheelMove = GetMouseWheelMove();
    dist *= pow(CameraMoveExponential, mouseWheelMove);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        mousePosPre = GetMousePosition();
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        mousePosNew = GetMousePosition();
        mousePosDelta.x = mousePosNew.x - mousePosPre.x;
        mousePosDelta.y = mousePosNew.y - mousePosPre.y;
        mousePosPre = mousePosNew;
        yaw += -SENSITIVITY * mousePosDelta.x;
        pit += -SENSITIVITY * mousePosDelta.y;
        pit = VAL_LIMIT(pit, -1.57, 1.57);
    }
    Vector3 vec = (Vector3){cosf(pit)*cosf(yaw), cosf(pit)*sinf(yaw), sinf(pit)};
    vec = Vector3Scale(vec, dist);
    camera->position = Vector3Subtract(camera->target, vec);
}
```
## 第一视角
WSAD分别控制视角的前后左右移动，EQ分别控制上下移动，滚轮控制移动速度。
```c
#include "camera.h"
#include <math.h>

#define CameraMoveSpeedInit              0.2f
#define VAL_LIMIT(x, min, max)           (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define MouseMoveSensitivity             0.003f
#define MouseScrolSensitivity            1.5f
#define CameraMoveExponential            1.2f

typedef struct {
    Vector3 r;
    Vector3 v;
} RadiusVelocity;

typedef enum {
    MOVE_FRONT = 0,
    MOVE_BACK,
    MOVE_RIGHT,
    MOVE_LEFT,
    MOVE_UP,
    MOVE_DOWN
} KeyMoves;

float pit, rol, yaw;
Vector2 previousMousePosition;
const int moveControl[6] = { 'W', 'S', 'D', 'A', 'E', 'Q' };
float CameraMoveSpeed = CameraMoveSpeedInit;

void SetCameraSpeed(float speed) {CameraMoveSpeed=speed;}
float GetCameraSpeed() { return CameraMoveSpeed; }

void Init_Camera(Camera *camera)
{
    camera->position = (Vector3){ 0.0f, -30.0f, 30.0f };
    camera->target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera->up = (Vector3){ 0.0f, 0.0f, 1.0f };
    camera->fovy = 45.0f;
    Vector3 delta = Vector3Subtract(camera->target, camera->position);
    pit = atan(delta.z / sqrt(delta.x*delta.x + delta.y*delta.y));
    rol = 0;
    yaw = atan2(delta.y, delta.x) - PI/2;
    pit = VAL_LIMIT(pit, -1.57f, 1.57f);
    previousMousePosition = GetMousePosition();
    CameraMoveSpeed = CameraMoveSpeed;
    DisableCursor();
}

void Update_Camera(Camera *camera)
{
    Vector2 mousePositionDelta;
    Vector2 mousePosition = GetMousePosition();
    float mouseWheelMove = GetMouseWheelMove();
    CameraMoveSpeed *= pow(CameraMoveExponential, mouseWheelMove);
    char direction[3] = {
        IsKeyDown(moveControl[MOVE_RIGHT]) - IsKeyDown(moveControl[MOVE_LEFT]),
        IsKeyDown(moveControl[MOVE_FRONT]) - IsKeyDown(moveControl[MOVE_BACK]),
        IsKeyDown(moveControl[MOVE_UP])    - IsKeyDown(moveControl[MOVE_DOWN]),
    };
    mousePositionDelta.x = mousePosition.x - previousMousePosition.x;
    mousePositionDelta.y = mousePosition.y - previousMousePosition.y;
    previousMousePosition = mousePosition;
    float dx = direction[0] * CameraMoveSpeed;
    float dy = direction[1] * CameraMoveSpeed;
    float dz = direction[2] * CameraMoveSpeed;
    camera->position.x += dx * cos(pit)*cos(yaw) - dy * cos(pit)*sin(yaw);
    camera->position.y += dx * cos(pit)*sin(yaw) + dy * cos(pit)*cos(yaw);
    camera->position.z += dz + dy*sin(pit);
    pit -= (MouseMoveSensitivity * mousePositionDelta.y);
    yaw -= (MouseMoveSensitivity * mousePositionDelta.x);
    pit = VAL_LIMIT(pit, -1.57f, 1.57f);
    camera->target.x = camera->position.x - cosf(pit)*sinf(yaw);
    camera->target.y = camera->position.y + cosf(pit)*cosf(yaw);
    camera->target.z = camera->position.z + sinf(pit);
}
```
# 画正方体线框
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/105035cd03fcf91e497d32341638fd2f.gif#pic_center)
```cpp
#include "camera.h"
typedef unsigned char  u8;
#define SIDEWIDTH      3

int main(void) {
    Camera camera;
    Vector3 Pstart, Pend;
    u8 state;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetTargetFPS(60);
    SetWindowMonitor(1);
    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitGraph(0, 0, "RayLib-3D");
	Init_Camera(&camera);

    while (!WindowShouldClose()) {
        Update_Camera(&camera);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                // DrawGrid(120, 6);
                for (u8 i = 0; i < 8; i++) {
                    Vector3 Pstart = (Vector3){
                        SIDEWIDTH * ((((i & 0x01) >> 0) << 1) - 1),
                        SIDEWIDTH * ((((i & 0x02) >> 1) << 1) - 1),
                        SIDEWIDTH * ((((i & 0x04) >> 2) << 1) - 1),
                    };
                    for (u8 j = 0; j < 3; j++) {
                        state = i ^ (1 << j);
                        Vector3 Pend = (Vector3){
                            SIDEWIDTH * ((((state & 0x01) >> 0) << 1) - 1),
                            SIDEWIDTH * ((((state & 0x02) >> 1) << 1) - 1),
                            SIDEWIDTH * ((((state & 0x04) >> 2) << 1) - 1),
                        };
                        DrawLine3D(Pstart, Pend, BLACK);
                    }
                }
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
    return 0;
}
```
# 球面上均匀取点
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e7ca2ee184c620e9a9b938b8e023a27f.gif#pic_center)

```cpp
#include "camera.h"
#include <ctime>
// #define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <iostream>
using namespace std;

class randEngine {
public:
    randEngine() {
        _gen.seed(time(0));
        _gen = std::default_random_engine((unsigned int)time(0));
        // _NormDis = std::normal_distribution<double>(0, 1);
        _UniFloatDis = std::uniform_real_distribution<double>(0, 1);
        // _UniIntDis = std::uniform_int_distribution<unsigned>(0, N - 1);
    }
    double rand01() { return _UniFloatDis(_gen); }
private:
    std::default_random_engine _gen;  // 生成初始化种子
    // std::normal_distribution<double> _NormDis;  // 正态分布
    std::uniform_real_distribution<double> _UniFloatDis;  // [0,1]均匀分布
    // std::uniform_int_distribution<unsigned> _UniIntDis;  // 整数均匀分布
};

int main(void) {
    Camera camera;
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetTargetFPS(60);
    SetWindowMonitor(1);
    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitGraph(0, 0, "RayLib-3D");
    Init_Camera(&camera);

    srand(time(0));
    vector<Vector3> points;
    double theta1, theta2;
    Vector3 vec;
    randEngine re;
    for (uint32_t i = 0; i < 1000; i++) {
        // theta1 = re.rand01()*M_PI-M_PI_2;
        theta1 = acos(re.rand01()*2-1)-M_PI_2;
        theta2 = re.rand01()*M_PI*2;
        vec.x = cos(theta1) * cos(theta2);
        vec.y = cos(theta1) * sin(theta2);
        vec.z = sin(theta1);
        points.push_back(vec);
    }

    while (!WindowShouldClose()) {
        Update_Camera(&camera);
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
            DrawGrid(120, 2);
            for (uint32_t i = 0; i < points.size(); i++)
                DrawSphere(points[i], 0.01, BLACK);
        EndMode3D();
        DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
    return 0;
}
```

