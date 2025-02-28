# Unreal 学习笔记

## 2023/12/10

### 简单的环境光照

1. 面板：窗口-环境光照混合器，添加除了体积云以外的光照系统

2. 添加后处理体积，设置无限范围（未限定）
3. 设置Exposure，保证brightness不会改变

<img src="uepics\ue00.png" alt="image-20231210132156671" style="zoom:50%;" />

4. 使用Gaea制作地形

5. 植物模式中刷树木和草丛，对应的几何体要转化成Nanite虚拟几何体。
6. 打开path tracing观察效果

## 2023/12/14 

### 传送门效果

1. **基本组件Actor**

<img src="uepics\01.png" alt="image-20231214143428945" style="zoom:67%;" />

2. 新建RenderTargetTexture, 用来获取场景捕获组件得到的图像信息

![image-20231214143552400](uepics\02.png)

3. 使用RenderTargetTexture创建材质，着色模型为无光照，并设置屏幕对齐uv坐标

<center>
    <img src="uepics\03.png" alt="image-20231214143644678" style="zoom:50%"/>
    <img src="uepics\04.png" alt="image-20231214143734338" style="zoom:50%"/>
</center>

4. 将材质赋给portal的静态网格体平面并取消平面的碰撞阻挡预设为“OverlapOnlyPawn”。
5. 编写传送逻辑，本质为带时延检测的Set Actor Location

![image-20231214144349648](uepics\05.png)

6. 控制RenderTargetTexture随相机而变动

![image-20231214144857212](uepics\06.png)

7. 如果传送门不是前后对应，需要重新生成平面材质，因为RenderTarget已经改变了（所以一定要注意门的朝向），RenderTarget是实时改变的！材质实例也会实时改变！

## 2023/12/19 PCG入门1

1. 打开PCG插件![image-20231219160651107](uepics\07.png)

2. 创建PCG图表，初始Input和Output。
3. 添加节点表面采样器，用来确定程序化生成表面（一般是某个地形）
4. 使用D打开调试观看选项
5. 添加点变换，投影变化，密度过滤器，噪音，初步生成分布图
6. 添加静态网格体生成器，选择需要使用的Static mesh和材质，进行程序化生成

![image-20231220153227290](uepics\08.jpg)

![image-20231219163030884](uepics\10.png)

![image-20231219162945330](uepics\09.png)

## 2023/12/20 PCG入门2

**使用样条线 产生控制点和路径。**

Spline actor,关闭“启用Tick并开始”，可以打开“缩放可视化宽度”预览路径大小。

PCG蓝图这里，“获取样条线数”据过滤选择所有世界Actor的话，需要设置过滤方式（通用：By Tags，手动为actor设置标签）。然后连接“样条线采样器”，尺寸和模式均可改变。设置内部距离衰减曲线用来调整密度衰减分布。

使用“距离”可以利用内部和样条线上的距离差异来固定密度衰减距离，“边界修改器”用来控制产生点的大小，用于后续subtract产生路径的操作（“difference”函数，密度函数这里要改成“二进制”）

![image-20231220154444124](uepics\11.png)

产生的程序化区域可以通过“投影”投射到地形表面，经过点变换，密度过滤，静态网格体生成就可以得到具体的森林。

![image-20231220155951664](uepics\12.png)

## 2023/12/27 PCG入门3

**点散布**

CreatePointsGrid用来生成点阵列网格（存在问题，勾选Local和去掉Cull points outside volume均会对复制点操作产生影响，目测自身产生点在世界原点。）

使用边界修改器限制大小，复制点将产生的点网格复制到目标点位。

最后依旧是分层过滤密度，此次处理加入了根据密度放缩大小的操作（scale by Density）。

![image-20231226145638508](uepics\13.png)

<center>
    <img src="uepics\15.png" alt="image-20231226150036818" style="zoom:46%;" />
    <img src="uepics\14.png" alt="image-20231226145948062" style="zoom:50%;" />

**封装与子图**

样条线最好不要重复。

“复制点”节点参数：![image-20231228155951090](uepics\16.png),“边界修改器”初始可视化数值可写为10。

## 2023/12/28 PCG入门4

**自定义PCG Actor**

将PCG图集成到actor中，可以用碰撞体或者静态网格体圈定生成范围，编译后即可得到PCG图的效果，并且该actor响应多个范围区域。

**设置属性暴露到蓝图中使用**

在actor中添加变量，打开眼睛开关将其暴露出去。在PCG蓝图中使用获取Actor属性来得到对应名字的变量。

![image-20231228162113827](uepics\17.png)

这里需要去掉勾选“始终重新查询Actors”以免每次参数变化后actor id都会改变导致不正确的生成和蓝图报错。

![image-20231228162214020](uepics\18.png)

## UE4/5 Niagara粒子系统特效

### 2024/4/10 小创烟雾与火焰

烟雾/云

[1Unreal Engine Niagara Cloud Tutorial(1080P_HD)_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1Mb411T7bC?p=8&vd_source=270ec920731a4632450bbb5c3407bd62)

火焰/燃烧

[【UE5Niagara】03使用Niagara制作火特效_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1FW4y1K7G3/?spm_id_from=333.337.search-card.all.click&vd_source=270ec920731a4632450bbb5c3407bd62)

[虚幻引擎Niagara第四课：Sprite入门特效-火焰_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1As4y1j7p6/?spm_id_from=333.788&vd_source=270ec920731a4632450bbb5c3407bd62)

### 2024/4/12 爆炸烟雾

一瞬间Burst500个粒子，并作为烟雾发射器的源。

![image-20240412114757339](uepics\19.png)

设置粒子颜色，生命周期和大小。

![image-20240412132914314](uepics\20.png)

设置粒子分布：sphere location

![image-20240412133046879](uepics\21.png)

使用点模式的粒子发射：

![image-20240412133142517](uepics\22.png)

添加-500的gravity force使粒子降落，添加Vector Noise Force和Curl Noise Force增加粒子的乱流，添加粒子大小衰减，使用二次曲线。

![image-20240412133324684](uepics\23.png)

最后加上碰撞。减少弹射，增加摩擦。

![image-20240412133428303](uepics\24.png)

自制粒子材质，自发光增至500。

![image-20240412133559024](uepics\25.png)

流体发射器部分：

![image-20240412133723350](uepics\26.png)

![image-20240412133802750](uepics\27.png)

去掉开放-z方向，使其与地面发生碰撞。

![image-20240412133847682](uepics\28.png)

重写烟雾颜色：

![image-20240412133918014](uepics\29.png)

爆炸后产生的乱流粒子，可以直接复制上面的发射器，修改参数如下即可。

![image-20240412134040444](uepics\30.png)

![image-20240412134105259](uepics\31.png)

![image-20240412134124973](uepics\35.png)

![image-20240412134133219](uepics\34.png)

![image-20240412134142091](uepics\33.png)

![image-20240412134148905](uepics\32.png)

蓄力效果：

![image-20240412134832012](uepics\36.png)

![image-20240412134907757](uepics\37.png)

爆炸及火花效果：

![image-20240412135010317](uepics\38.png)

### 2024/4/12 Fluid 传送门

![image-20240412142137979](uepics\39.png)

核心实现：Vortex Force

![image-20240412142241068](uepics\40.png)

### 2024/4/14 Fluid龙卷风

![image-20240414103531169](uepics\41.png)

shape Location决定龙卷风形状，涡流速度提供旋转，与烟雾发射器结合。

![image-20240414103642014](uepics\42.png)

![image-20240414103717013](uepics\43.png)

![image-20240414103801862](uepics\44.png)

## Niagara SPH流体模拟

> [!IMPORTANT]
>
> *写在前面：*
>
> *关于水渲染：*
>
> *物理层面：控制方程，无散流场，FFT模拟，拉格朗日/欧拉方法*
>
> *渲染层面：材质基础，反射与折射，次表面散射，Path guiding*

### 粒子模拟阶段

**Neighbor Grid 3D 相邻网格**：

![image-20240429123759855](uepics\45.png)

包括每个单元格最大查询邻居数量，每个轴上有多少单元格。

接下来需要一个世界和网格体的转换矩阵，对于一个三维网格，我们Local空间的坐标从0-1，因此需要构建世界到Unit单位空间的转换矩阵。实际上就是Transform、Rotation、Scale三个变换矩阵和TLocal和RLocal两个本地空间变换矩阵。

对于WorldToUnit矩阵：

```glsl
T = float4x4(
    float4(1.0f, 0.0f, 0.0f, 0.0f),
    float4(0.0f, 1.0f, 0.0f, 0.0f),
    float4(0.0f, 0.0f, 1.0f, 0.0f),
    -Offset, 1.0f);//世界空间偏移
S = float4x4(
    float4(1./Scale.x, 0.0f, 0.0f, 0.0f),
    float4(0.0f, 1./Scale.y, 0.0f, 0.0f),
    float4(0.0f, 0.0f, 1./Scale.z, 0.0f),
    float4(0.0f, 0.0f, 0.0f, 1.0f));
R=transpose(Rotation);//世界空间旋转矩阵（欧拉角）
TLocal=float4x4(
    float4(1.0f, 0.0f, 0.0f, 0.0f),
    float4(0.0f, 1.0f, 0.0f, 0.0f),
    float4(0.0f, 0.0f, 1.0f, 0.0f),
    -PivotVector, 1.0f);
PivotVector=(1-LocalPivot)-1;//组件中心点
Rlocal=transpose(float4x4(
    float4(1.0f, 0.0f, 0.0f, 0.0f),
    float4(0.0f, 1.0f, 0.0f, 0.0f),
    float4(0.0f, 0.0f, 1.0f, 0.0f),
    float4(0.0f, 0.0f, 0.0f, 1.0f));)

    ret = mul(T, R);
ret = mul(ret, RLocal);
ret = mul(ret, S);
ret = mul(ret, TLocal);
WorldToUnit=ret;
//ret=(mul(mul(mul(T,R),RLocal),S),TLocal);//倒过来乘就是UnitToWorld
```

得到初始的NeighbourGrid和WorldToUnit后，就可以对3D网格cell进行编号：

```glsl
//Input:粒子位置Position
//网格对象NeighborGrid
//转换矩阵WorldToUnit
//当前执行块的ID：InstanceIdx来自于ExecutionIndex

#if GPU_SIMULATION
float3 UnitPos;
NeighborGrid.SimulationToUnit(Position, SimulationToUnit, UnitPos);//得到局部坐标
int3 index;
NeighborGrid.UnitToIndex(UnitPos, index.x, index.y, index.z);//局部坐标转换为线性编号（(0,0,0)对应1；(1,0,0)对应2；(2,0,0)对应3；(0,1,0)对应NumCells.x+1；(0,0,1)对应NumCells.y*NumCells.x+1）
int3 NumCells;
NeighborGrid.GetNumCells(NumCells.x, NumCells.y, NumCells.z);
int MaxNeighborsPerCell;
NeighborGrid.MaxNeighborsPerCell(MaxNeighborsPerCell);
float3 CellSize;
NeighborGrid.GetCellSize(CellSize);

if (index.x >= 0 && index.x < NumCells.x && index.y >= 0 && index.y < NumCells.y && index.z >= 0 && index.z < NumCells.z)//只对在网格内的粒子编号
{
    int Linearindex;
    NeighborGrid.IndexToLinear(index.x, index.y, index.z, Linearindex);
    //根据之前算的Index来求该粒子在哪个小网格内并返回该网格的id
    int PreviousNeighborCount;
    NeighborGrid.SetParticleNeighborCount(Linearindex, 1, PreviousNeighborCount);
    //给网格内的粒子进行编号，每次增1
    if (PreviousNeighborCount < MaxNeighborsPerCell)
    {//只计算Max以内的粒子
        int NeighborGridLinear;
        NeighborGrid.NeighborGridIndexToLinear(index.x, index.y, index.z,PreviousNeighborCount, NeighborGridLinear);
        //如果一个区内的粒子数不满足最大容量，那么多余的编号保留，他的邻居单元格从MaxNeighborsPerCell开始继续编号（比如网格1内有3个粒子，编号为0，1，2，假设Max=8,那么其邻居网格2内例粒子编号为8，9，10...）
        int IGNORE;
        NeighborGrid.SetParticleNeighbor(NeighborGridLinear, InstanceIdx, IGNORE);
        //将当前粒子的ID和网格索引一一对应起来。
    }
}
#endif
```

根据SPH流体理论开始计算密度和压力：

粒子属性：Density、Pressure

输入：

- NeighborGrid：相邻3D网格
- Particle Attribute Reader：粒子属性阅读器
- PointMass：点质量
- WorldGridExtent：世界网格缩放
- WorldToGridUnit：转换矩阵（SimulationToUnit）
- Rest Density：静止密度，从密度中减去 Rest Density意味着当密度高于静止密度时，压力将是正的，从而产生排斥力；低于静止密度时，压力将为负值，从而产生吸引力。使得粒子从高压区域移动到较低压力区域。（同时也负责表面张力效果）。
- GasCosntantK：气体常数K，用来表示液体的可压缩性，数值越大越不可压缩，这也是流体的重要特性之一。
- Position：粒子位置
- SelfParticleIndex：粒子的id

输出：

- Density
- Pressure

```glsl
Density = 0;
Pressure = 0;

#if GPU_SIMULATION
const int3 IndexOffsets [ 27 ] = 
{
    int3(-1,-1,-1),
    int3(-1,-1, 0),
    int3(-1,-1, 1),
    int3(-1, 0,-1),
    int3(-1, 0, 0),
    int3(-1, 0, 1),
    int3(-1, 1,-1),
    int3(-1, 1, 0),
    int3(-1, 1, 1),

    int3(0,-1,-1),
    int3(0,-1, 0),
    int3(0,-1, 1),
    int3(0, 0,-1),
    int3(0, 0, 0),
    int3(0, 0, 1),
    int3(0, 1,-1),
    int3(0, 1, 0),
    int3(0, 1, 1),

    int3(1,-1,-1),
    int3(1,-1, 0),
    int3(1,-1, 1),
    int3(1, 0,-1),
    int3(1, 0, 0),
    int3(1, 0, 1),
    int3(1, 1,-1),
    int3(1, 1, 0),
    int3(1, 1, 1),
};

// Derive the Neighbor Grid Index from the world position
float3 UnitPos;
NeighborGrid.SimulationToUnit(Position, SimulationToUnit, UnitPos);

int3 Index;
NeighborGrid.UnitToIndex(UnitPos, Index.x,Index.y,Index.z);

int3 NumCells;
NeighborGrid.GetNumCells(NumCells.x, NumCells.y, NumCells.z);

// loop over all neighbors in this cell
int MaxNeighborsPerCell;
NeighborGrid.MaxNeighborsPerCell(MaxNeighborsPerCell);

float3 CellSize = WorldGridExtent / NumCells;

float DesitySum = 0;
float SmoothRadius = CellSize.x;  // In UE, the length unit is cm, but now just see it as m
float h2 = SmoothRadius * SmoothRadius;

float KernelPoly6 = 315.0 / (64.0 * 3.141592 * pow(SmoothRadius, 9));  // 1/m^9

float temp_max = 0;
for(int xxx = 0; xxx < 27; ++xxx)
{//遍历周围3*3*3的相邻网格
    for(int i = 0; i < MaxNeighborsPerCell; i++)
    {//遍历计算粒子
        const int3 IndexToUse = Index + IndexOffsets[xxx];
        
        int NeighborLinearIndex;
        NeighborGrid.NeighborGridIndexToLinear(IndexToUse.x, IndexToUse.y, IndexToUse.z, i, NeighborLinearIndex);
        
        int CurrNeighborIdx;
        NeighborGrid.GetParticleNeighbor(NeighborLinearIndex, CurrNeighborIdx);
        
        // temp bool used to catch valid/invalid results for direct reads
        bool myBool; 
        float3 OtherPos;
        DirectReads.GetVectorByIndex<Attribute="Position">(CurrNeighborIdx, myBool, OtherPos);

        const float3 vectorFromOtherToSelf = Position - OtherPos;//位置梯度
        const float r = length(vectorFromOtherToSelf);  // 距离distance，In UE, the length unit is cm, but now just see it as m
        const float h2_r2 = h2 - r * r;

        if(IndexToUse.x >= 0 && IndexToUse.x < NumCells.x &&
           IndexToUse.y >= 0 && IndexToUse.y < NumCells.y &&
           IndexToUse.z >= 0 && IndexToUse.z < NumCells.z &&
           CurrNeighborIdx != -1 && r < SmoothRadius&& CurrNeighborIdx != SelfParticleIndex)
        {
            DesitySum += pow(h2_r2, 3);      //密度积累方式  
        }
    }
}

Density = KernelPoly6 * PointMass * DesitySum;  // kg/m^3
Pressure = max((Density - RestDensity) * GasConstantK, 0);  // N/m^2
//压力计算？：pow((Density / RestDensity), 7) 
#endif
```

计算完力之后，需要将力转化为加速度，要保证符合牛顿第二第三定律：

粒子属性：Density、Pressure、Mass、Viscosity、Velocity

输入：

- NeighborGrid：相邻3D网格
- Particle Attribute Reader：粒子属性阅读器
- PointMass：点质量
- WorldGridExtent：世界网格缩放
- WorldToGridUnit：转换矩阵（SimulationToUnit）
- Rest Density：静止密度，从密度中减去 Rest Density意味着当密度高于静止密度时，压力将是正的，从而产生排斥力；低于静止密度时，压力将为负值，从而产生吸引力。使得粒子从高压区域移动到较低压力区域。（同时也负责表面张力效果）。
- GasCosntantK：气体常数K，用来表示液体的可压缩性，数值越大越不可压缩，这也是流体的重要特性之一。
- Position：粒子位置
- SelfParticleIndex：粒子的id
- Viscosity：粘度，流体粘度用来衡量流体运动的阻力
- Desnity：区域密度
- Pressure：压力
- Velocity：粒子速度（vecotor3）

输出：

- ViscosityAcc：阻力加速度
- PressureAcc：压力加速度

或者输出：

- OutAcceleratio：加速度之和

```glsl
OutAcceleration = float3(0,0,0);

#if GPU_SIMULATION
const int3 IndexOffsets [ 27 ] = 
{
    int3(-1,-1,-1),
    int3(-1,-1, 0),
    int3(-1,-1, 1),
    int3(-1, 0,-1),
    int3(-1, 0, 0),
    int3(-1, 0, 1),
    int3(-1, 1,-1),
    int3(-1, 1, 0),
    int3(-1, 1, 1),

    int3(0,-1,-1),
    int3(0,-1, 0),
    int3(0,-1, 1),
    int3(0, 0,-1),
    int3(0, 0, 0),
    int3(0, 0, 1),
    int3(0, 1,-1),
    int3(0, 1, 0),
    int3(0, 1, 1),

    int3(1,-1,-1),
    int3(1,-1, 0),
    int3(1,-1, 1),
    int3(1, 0,-1),
    int3(1, 0, 0),
    int3(1, 0, 1),
    int3(1, 1,-1),
    int3(1, 1, 0),
    int3(1, 1, 1),
};

// Derive the Neighbor Grid Index from the world position
float3 UnitPos;
NeighborGrid.SimulationToUnit(Position, SimulationToUnit, UnitPos);

int3 Index;
NeighborGrid.UnitToIndex(UnitPos, Index.x,Index.y,Index.z);

int3 NumCells;
NeighborGrid.GetNumCells(NumCells.x, NumCells.y, NumCells.z);

// loop over all neighbors in this cell
int MaxNeighborsPerCell;
NeighborGrid.MaxNeighborsPerCell(MaxNeighborsPerCell);

float3 CellSize = WorldGridExtent / NumCells;  // cm

float SmoothRadius = CellSize.x;  // In UE, the length unit is cm, but now just see it as m
float h2 = SmoothRadius * SmoothRadius;  // m^2

float KernelSpiky = -45.0 / (3.141592 * pow(SmoothRadius, 6));  // 1/m^6
float KernelViscosity = 45.0 / (3.141592 * pow(SmoothRadius, 6));  // 1/m^6

for(int xxx = 0; xxx < 27; ++xxx)
{
    for(int i = 0; i < MaxNeighborsPerCell; i++)
    {
        const int3 IndexToUse = Index + IndexOffsets[xxx];
        
        int NeighborLinearIndex;
        NeighborGrid.NeighborGridIndexToLinear(IndexToUse.x, IndexToUse.y, IndexToUse.z, i, NeighborLinearIndex);
        
        int CurrNeighborIdx;
        NeighborGrid.GetParticleNeighbor(NeighborLinearIndex, CurrNeighborIdx);
        
        if (SelfParticleIndex == CurrNeighborIdx)
        {
            continue;
        }
        bool myBool; 
        float3 OtherPos;//邻居粒子的位置
        DirectReads.GetVectorByIndex<Attribute="Position">(CurrNeighborIdx, myBool, OtherPos);
        float OtherDensity;//邻居粒子的密度
        DirectReads.GetFloatByIndex<Attribute="Density">(CurrNeighborIdx, myBool, OtherDensity);
        float OtherPressure;//邻居粒子的压力
        DirectReads.GetFloatByIndex<Attribute="Pressure">(CurrNeighborIdx, myBool, OtherPressure);
        float3 OtherVel;//邻居粒子的速度
        DirectReads.GetVectorByIndex<Attribute="Velocity">(CurrNeighborIdx, myBool, OtherVel);

        const float3 vectorFromOtherToSelf = Position - OtherPos;
        const float3 dirFromOtherToSelf = normalize(vectorFromOtherToSelf); 
        const float r = length(vectorFromOtherToSelf); 

        if(IndexToUse.x >= 0 && IndexToUse.x < NumCells.x &&
           IndexToUse.y >= 0 && IndexToUse.y < NumCells.y &&
           IndexToUse.z >= 0 && IndexToUse.z < NumCells.z &&
           CurrNeighborIdx != -1 && r < SmoothRadius)
        {
            const float h_r = SmoothRadius - r; // m
            const float h2_r2 = h2 - r * r; // m^2

            // F_Pressure
            const float3 pterm = -PointMass * KernelSpiky * h_r * h_r * (Pressure + OtherPressure) / (2.f * Density * OtherDensity) * dirFromOtherToSelf;

            // F_Viscosity
            const float3 vterm = PointMass * KernelViscosity * Viscosity * h_r * (OtherVel - Velocity) / (Density * OtherDensity);

            OutAcceleration += pterm + vterm;  // m/s^2
        }
    }
}
#endif
```

将加速度变成速度和位置信息，并添加重力和包围盒：

输入：

- DeltaTime：单位时间，由引擎提供
- Gravity：重力，9.8N/kg，加上方向设为-980
- Acceleration：合加速度
- Velocity：速度
- Position：位置
- WordlGirdExtent：世界网格缩放
- MaxAccel：最大加速度
- MaxVelocity：最大速度
- WallDmaping：反弹之后的速度，“墙的粘滞性”

```glsl
#pragma once

DeltaTime = DeltaTime;

OutVelocity = float3(0.0, 0.0, 0.0);
OutPosition = float3(0.0, 0.0, 0.0);

#if GPU_SIMULATION

Acceleration += Gravity;  // m/s^2

// acceleration limit
if(length(Acceleration) > MaxAccel)
{//加速度限制
    Acceleration = normalize(Acceleration) * MaxAccel;  // m/s^2
}

// advent
Velocity += Acceleration * DeltaTime;  // m/s

// velocity limit
if(length(Velocity) > MaxVelocity)
{//速度限制
    Velocity = normalize(Velocity) * MaxVelocity;  // cm/s
}

Position += Velocity * DeltaTime;  // cm

if(Position.x < -WorldGridExtent.x/2)
{
    Velocity.x *= WallDamping;
    Position.x = -WorldGridExtent.x/2;
}
if(Position.x > WorldGridExtent.x/2)
{
    Velocity.x *= WallDamping;
    Position.x = WorldGridExtent.x/2;
}
if(Position.y < -WorldGridExtent.y/2)
{
    Velocity.y *= WallDamping;
    Position.y = -WorldGridExtent.y/2;
}
if(Position.y > WorldGridExtent.y/2)
{
    Velocity.y *= WallDamping;
    Position.y = WorldGridExtent.y/2;
}
if(Position.z < -WorldGridExtent.z/2)
{
    Velocity.z *= WallDamping;
    Position.z = -WorldGridExtent.z/2;
}
if(Position.z > WorldGridExtent.z/2)
{
    Velocity.z *= WallDamping;
    Position.z = WorldGridExtent.z/2;
}

OutPosition = Position;  // cm
OutVelocity = Velocity;  // cm

#endif // GPU_SIMULATION
```

![sphwater](uepics\sphwater.gif)
