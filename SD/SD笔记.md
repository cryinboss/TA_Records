# Substance Designer

## 1.0 节点基础
**材质工作流**
高度图是一切的起点，用来生成法线贴图，环境遮蔽贴图，粗糙度贴图等，然后生成Albedo基础颜色图，进行各种材质的混合，得到最终的输出。
<img src="sdpics\workflow_material1.jpg" alt="Alt text" />

- 右键包链接3D场景，可以把预览界面的模型换成自己的建模。
- **Tessellation Shader:**

Tessellation Shader是一种在图形渲染管线中用于细分几何体的着色器。它可以在渲染时动态地增加模型的多边形数量，从而提高模型的细节。

Tessellation Shader主要有三个阶段：

1. **控制着色器**(Control Shader)：这个阶段主要负责计算细分的级别，也就是要增加多少新的顶点。这通常会根据模型距离摄像机的远近来决定，模型离摄像机越近，细分的级别就越高。

2. **细分阶段**(Tessellation Primitive Generator)：这个阶段是硬件级别的，会根据控制着色器的输出来生成新的顶点。

3. **评估着色器**(Evaluation Shader)：这个阶段会计算新生成的顶点的位置和其他属性，如法线，纹理坐标等。

使用Tessellation Shader的好处是可以在不增加模型文件大小的情况下提高模型的细节。但是，它也会增加渲染的计算负担，因此需要根据具体的需求和硬件性能来适当使用。


## 1.1 各级细节制作精要
- **Large Shapes**:
<img src="sdpics\large_shapes-1.jpg" alt="Alt text" style="margin-left:25%" />
使用初级纹理（如泊林噪音纹理)进行材质大体感官上的定性


- **Medium Shapes**:
需要制作一些新的图形时，使用到Shape节点，tile可以调节数量，使用其他节点对其变形以达到理想效果。
使用levels,blend,blur节点调节效果强度，使用slope相关节点进行形状的采样，在shape slpatter节点中增加对应的槽数量，调节采样数量以适应原先的材质。
比较重要的**属性**：
Size:形状大小，随机性，可使用map进行调节
Position:位置随机性，可使用map进行调节
Rotation:旋转随机性，可使用map进行调节
Height:高度随机性，可使用map进行调节
Masking:遮罩控制数量和范围，可使用map进行调节
Shape scale:
<img src="sdpics\m_shapes-1.jpg" alt="Alt text" />

- **Small Shapes**:
<img src="sdpics\s_shapes-1.jpg" alt="Alt text" />
是中级细节进一步的细化

## 1.2 节点搭配与使用
常用节点：
- blend:混合，很重要，混合模式有很多，覆盖，相乘，投影，叠加等。
- levels:筛选输入输出的暗部和亮部
- blur:有高精度的blur和低精度的blur,使用opacity属性调节
- warp：扭曲变形，还有directional warp用来径向变形
- transform 2d:对图形进行变换
- Histogram Range:平均纹理的暗部和亮部，Postion调整明暗情况，Range决定使用原图色调的范围
- Slope blur grayscale:需要一张效果纹理来决定径向模糊的位置
- shape splatter:将做好的形状在高度图上进行**散布**，输出口splatter data可用来后续制作Albedo颜色。
- shape splatter blender color:制作材质基础色。需要设定基础背景色（层级最底层的环境颜色），
shape patter与前面的shape splatter对应即可。
- gradient map:将高度图转为颜色的神器，并且可以通过对实际物体图片（reference img）取色轻松制作颜色渐变曲线上色。
- highpass grayscale:理解为高通滤波，将灰度图“扁平化”

Expose as new graph input:将某一属性从局部区域提出来放入整个graph的变量调整区，暴露出来给user使用，常用于制作完次级物体纹理将其参数提出以便通过简单的调整可以得到不同效果的变体。
output:添加output节点，该材质暴露给外界使用。

## 1.3 Roughness map制作
<img src="sdpics\roughness-1.jpg" alt="Alt text" />
高度图生成AO图，反转后调整明暗强度，通过混合其他效果图再Histogram Range平均得到一张粗糙度贴图。

## 1.4 颜色制作
1. Albedo基础色
normal->curvature smooth->blend其他纹理->gradient Map拾取颜色，example as below:
<img src="sdpics\ground color.jpg" alt="Alt text" />
2. 对于每一级物体：**shape splatter blend color**
输入：
Background Color：Albedo基础色
Pattern：模式，与形状数量对应，gradient map 连接，纹理细节可以通过各种噪音纹理进行blend得到
Splatter Data1&2：与Shape Splatter的相应接口连接，传递分布属性的。
example as below:
<img src="sdpics\marble color.jpg" alt="Alt text" />
对于自身已经有细节纹理的物体，在生成gradient map时，可以直接使用形状shape，如下图实例twig树枝
<img src="sdpics\twig.jpg" alt="Alt text" style="margin-left:20%"/>
采取一形多用策略：(左侧全部都连接原shape)
<img src="sdpics\twig_color.jpg" alt="Alt text" /><img src="sdpics\dirt_part.jpg" alt="Alt text" />如此便得到了具有纹理细节的物体颜色。
3. 材质混合，通常在使用base material时需要用到，用来生成最终需要的贴图，包括**Base color,Normal,Roughness,Height,Ambient occlusion**,这里需要一张grayscale mask，灰度遮罩，**遮罩强度和原生颜色保留度成反比**。
<img src="sdpics\dirt_blend.jpg" alt="Alt text" />
在实例中，我们混合了雪和水材质，最终得到如下效果：
<img src="sdpics\ground_basecolor.jpg" alt="Alt text" /><img src="sdpics\dirt_final.jpg" alt="Alt text" />

## 2.0 练习
### 2.0.0 科幻宇宙甲
<img src="sdpics\armorbox.jpg" alt="Alt text" />

**要点**

- 镜像制作对称细节：mirror
- Tile generator进行纹理重复
- bevel 倒角制作羽化边缘或减少高度变化剧烈程度
- 除了blend不要忘记使用height blend进行高度图混合
- safe transform grayscale可以用来对高度图进行变换，有时候可以解决transform的一些问题
- emissive自发光节点需要手动导入
- noise贴图产生锈斑纹理
### 2.0.1 熔岩地面
<img src="sdpics\lava0.jpg" alt="Alt text" />

<img src="sdpics\lavaball.jpg" alt="Alt text" />

**要点**

- 生成元素：cell4
- edge detect 区分三级岩石细节
- noise + slope blur进行岩石形体扭曲
- 地面纹理决定岩浆纹理

### 2.0.2 幸运草丛
<img src="sdpics\clover_ground0.jpg" alt="Alt text" />
<img src="sdpics\clover_ground.jpg" alt="Alt text" />

**要点**

- 生成元素：polygon1 三角形+圆形组成叶片
- grdient linear1混合制造中心深，叶片边缘浅
- splatter circular制造不同数目的叶丛
- Tile Sampler:可替代shape splatter 的图形分布器
- 混合cloud纹理制作多级丛生叶丛（large shapes and small shapes）
- 上色方式：直接上色，curvature smooth上色，curvature sobel 上色，多级混合

### 2.0.3 砖石纹理

<img src="sdpics\baysalt_blcok.jpg" alt="baysalt_blcok" />\

<img src="sdpics\baysalt_block0.jpg" alt="baysalt_block0" />\

**要点**

- 生成元素：cube3D +Shape Extrude->噪音directional warp
- splatter砖块散布
- fluid液体噪音纹理+slope blur：扭曲形状
- cell4 +edge detect+slope blur：表面裂纹
- AO反转制作缝隙处颜色遮罩
- 上色方式：直接上色，curvature smooth上色，curvature sobel 上色，多级混合

### 2.0.4 蘑菇石地

<img src="sdpics\mushroom_ground.jpg" alt="mushroom_ground" style="zoom:50%"/>

<img src="sdpics\mushroom_ground1.jpg" alt="mushroom_ground1" style="zoom:50%" />

要点：

- 双tile sampler 制作石块
- histogram scan频繁使用，HBAO+histogram scam+invert grayscale制作遮罩贴图
- waveform +tile sampler制作叶片
- shape+shape extrude制作蘑菇和蘑菇柄，splatter circular制作蘑菇的褶皱和蘑菇群落
- mask map threshold可以用来调整遮罩效果

### 2.0.5 石头草地

<center>
    <img src="sdpics\grass_rock.jpg" alt="grass_rock" style="zoom:50%"/>
    <img src="sdpics\grass_rock1.jpg" alt="grass_rock1" style= "zoom:55.5%" />
</center>

要点：

- cell1制作石头的纹路
- polygon2制作石头的形状
- waveform+gradient linear制作叶片
- highpass+gradient map制作石头上的苔藓细节

### 2.0.6 苔藓沼泽地

<center>
    <img src="sdpics\moss0.jpg" alt="moss0" style="zoom:62.5%" />
    <img src="sdpics\moss1.jpg" alt="moss1" style="zoom:40%"/>
</center>

要点：

- 高斯滤波+fur1 blur +warp得到底部的杂乱苔藓纹理
- ploygon blend+slope blur+directional warp控制苔藓形状
- polygon2+shape mapper制作表面苔藓丛形状，利用1中的基础形状+histogram scan得到遮罩

### 2.0.7 甲片护盾

<img src="sdpics\shield0.jpg" alt="shield0" style="zoom:67%;" />

要点：

- shape+skew+mirror 形成盾甲形状，shape制作铁环，铁杆形状
- 逐级细节把控，large to samll
  1. 甲片凹陷（tile sampler），磨损(tile sampler)，刻痕(wave form 1+tile sampler)
  2. 铁环凹陷，磨损，刻痕
  3. histogram+flood fill + ( flood fill Bbox size) + flood fill random grayscale随机散布甲片颜色差异

- distance可以剔除出更干净的遮罩。
- fractal sum base+ slope blur + directional warp + vector warp grayscale + HBAO,warp + histogram scan 制作玻璃状扭曲



### 2.0.8 骸骨遗产

<img src="sdpics\bone_relic.jpg" alt="bone_relic" style="zoom:67%;" />

要点：

- splatter circular制作脊骨
- waveform+tile sampler制作骨头裂纹
- warp+directional warp制作地形
- 分层着色



### 2.0.9 沁槐沙漠

<img src="sdpics\stylized_sand.jpg" alt="stylized_sand" />

要点：

- wave form+tile sampler +directional warp制作波浪形沙漠纹理
- waveform+tile sampler分层制作叶片（可见和部分掩埋）
- shape->skew->mirror->bevel+splatter circular制作叶片/花瓣(中间可以混合一个shape，gradient制作叶脉)
- 分层着色，区域着色



### 2.0.10 闪耀水晶

<img src="sdpics\shing_crystal.jpg" alt="shing_crystal" />

要点：

- 多个cube3D交错形成棱柱纹理
- 多层着色形成混合斑斓的颜色，添加emissive自发光



### 2.0.11 老树盘根

<img src="sdpics\roots.jpg" alt="roots" />

要点：

- 多个cell4 + warp做树根交错

### 2.0.12 图腾石柱

<img src="sdpics\saman_graph.jpg" alt="saman_graph" />

要点：

- ring number=2 的 splatter circular制作图纹
- flood fill+flood fill to gradient +blend+bevel+blend切角+bevel+histogram scan重复此操作修正石砖形状
- slope blur + perlin noise制作石砖边缘歪斜
- 石砖表面不平整使用cloud noise制作
- 石砖表面凹坑使用perlin noise+ level保留部分高频细节制作

### 2.0.13 公园石砖路*

<img src="sdpics\02.jpg" alt="image-20231210202456510" />

### 2.0.14 燕戈沙垣

<img src="sdpics\03.jpg" alt="image-20231213132920613" />

要点：

- cell1+anisotropic noise制作岩石
- edge detect+flood fill+flood fill to gradient+warp制作三级岩突
- cloud noise+slope blur grayscale制作岩石边缘侵蚀
- 最后叠加横竖方向的anisotropic noise岩石分层
- 着色技巧：uber emboss可以进行烘焙照明式基于高度图的假着色

### 2.0.15 巢木苔穴

<img src="sdpics\04.png" alt="image-20231213133959131" />

要点：

- wave form+tile sampler制作木头表面扭曲纹理和边缘侵蚀
- wave form 制作树枝形态（叶片底座），连接到tile sampler的mask map
- 着色时需突出表面纹理（木头的峰线谷区）增加风格化感觉

### 2.0.16 旧造物牢笼

<img src="sdpics\05.png" alt="image-20231215191419301" />

要点：

- wave form+tile sampler制作划痕
- gradient_axial_reflected制作交叉造型
- stripes制作交叉网格和透明度贴图（Opacity）
- roughness贴图也可以用来制作光滑处的颜色以增强表现力
- 多重histogram scan制作强化锈迹制作成金属度贴图

### 2.0.17 皑皑雪崖

<img src="sdpics\06.png" alt="image-20231216204519011" />

要点：

- 大小tile sampling  subtract制作岩石
- flood fill->flood fill to random gradyscale->distance可以制作类cell纹理
- cell + slope blur + directional warp 制作裂痕
- normal +split G通道提取雪层
- 各级细节着色（岩石纹路，岩石斑驳色，雪分层）

### 2.0.18 异色漩涡

<img src="sdpics\07.png" alt="image-20231218135655915" />

要点：

- tile sampler 铺满+flood一次性制作石墙
- flood fill gradient + distance + levels制作部分石块的缺损。
- splatter circular制作旋转云朵纹理，既可以slope blur+high pass 制作皮类衣物质感，又可以swirl成漩涡

- 漩涡纹理的逐层混合，最后添加上polygon2星星

### 2.0.19 赛博箱

<img src="sdpics\08.png" alt="image-20231219105117508" style="zoom:67%;" />

要点：

- 形状制作，几乎相当于多边形建模

### 2.0.20 风雪夜堡

<img src="sdpics\09.png" alt="image-20231220190733270" style="zoom:67%;" />

要点：

- polygon2+splatter制作蚀痕
- Normal+light,0.5+的height level提取上层表面，本例中用来着色雪

### 2.0.21 生漆金属球

<img src="sdpics\10.png" alt="image-20231220200505718" style="zoom:67%;" />

要点：

- skew制作叶子作为原型，tile sampler极密集和极大的y轴缩放制作自上而下的油制污染

- RGBA split的G通道作为mask map，混合gaussian，调整tile sampler缩放和mask threshold参数制作三层油漆




### 2.1.0 破碎流沙石砖

<img src="sdpics\11.png" alt="11" style="zoom:50%;" />

要点：

- flood fill系列的用法复习

<img src="sdpics\flood_fill_inital.png" alt="image-20240116155454334" />

<img src="sdpics\flood_fill_edge_corruption.png" alt="image-20240116155227543" />

- 之前制作波浪纹理使用的是：**wave form+tile sampler +directional warp**，本练习中就地取材，四个石块tile sampler旋转45°，non-uniform blur+2 directional warp+ warp得到，更柔和可以利用normal + curvature系列调整边缘。


### 2.1.1 风格化屋顶

<center>
    <img src="sdpics\12.png" alt="image-20240119194347986" style="zoom:35%;" />
    <img src="sdpics\12_1.png" alt="image-20240119194522225" style="zoom:47.3%;" />
</center>

要点：

- 风格化边缘材质

<img src="sdpics\helper0.png" alt="image-20240119194721865" style="zoom: 50%;" />

- normal+rgba split 取不同通道为不同边缘着色，如G通道存储z轴负向。
- 木头裂纹通常可以用不同级crystal1和directional warp做规则扭曲。
- 同2.0.15制作树枝+叶片的散步，mask贴图使用histogram scan扫描出缝隙
- flood fill 增添风格化着色细节

![image-20240119195339069](sdpics\helpers1.png)

### 2.1.2 雪国铁轨

<img src="sdpics\13.png" alt="image-20240125220916214" style="zoom: 33%;" />

要点：

- 用level控制不同高度的纹理进行混合，比如：混合铁轨时，木板在下（30%），铁轨在上（70%），用height blend效果其实差不多

- 风格化积雪制作方式：

  1. 混合不同的噪声纹理

  <img src="sdpics\helper2.png" alt="image-20240125212933876" style="zoom:50%;" />

  2. 在与物体接触处扭曲调整

  <img src="sdpics\helper3.png" alt="image-20240125213044152" style="zoom:50%;" />

  2. 使用分型制作雪层叠的效果,最后使用high pass过滤高亮细节，再提取normal ，利用curvature smooth可以暗化层级交界处增强雪的分层感觉。

  <img src="sdpics\helper4.png" alt="image-20240125213142643" style="zoom:50%;" />

- 利用normal + curvature smooth + add blend三板斧丰富着色细节。

### 2.1.3 雪地碎砖

结合2.1.0石砖制作方式和2.1.2积雪制作方式的练习。

<img src="sdpics\14.png" alt="image-20240126173715807" style="zoom: 53.5%;" />

### 2.1.4 凛雪峭壁

<img src="sdpics\15.png" alt="image-20240207224650945" style="zoom: 64%;" />

**要点：**

- 圆角矩形+tile sampler subtract 制作石柱
- directional warp制作横向石柱的各级塌陷
- 风格化着色技巧

### 2.1.5 绳木组合

<img src="sdpics\16.png" alt="image-20240208210310250" style="zoom: 60%;" />

**要点：**

- 绳子的制作方式：

wave + tile sampler制作一节，wave形成绳子每一根线的纹理，得到一节后再次tile sampler，输入X=1,Y=N的方式，旋转，放缩制作绳子，角度在60-120之间合适的。

<center>
    <img src="sdpics\helper5.png" alt="image-20240208210708617" style="zoom: 60%;"/>
    <img src="sdpics\helper6.png" alt="image-20240208210933100" style="zoom: 40%;"/>
</center>

由此得到一根绳子后可以继续得到不同的绳子组合。

<img src="sdpics\helper7.png" alt="image-20240208211308471" style="zoom:67%;" />

### 2.1.6 木板挂帘

<img src="sdpics\17.png" alt="image-20240304135044445" style="zoom: 50%;" />

**要点：**

- 窗帘的制作方式，directional blur+warp

  <center>
      <img src="sdpics\helper8.png" alt="image-20240304135154122" style="zoom:33%;" />
      <img src="sdpics\helper9.png" alt="image-20240304135239287" style="zoom: 33%;" />
  </center>

### 2.1.7 松软一些的雪地

<img src="sdpics\24.png" alt="image-20240312205459295" style="zoom:50%;" />

**要点：**

<img src="sdpics\helper14.png" alt="image-20240312205543781" style="zoom:67%;" />

### 2.1.8 固化一些的雪地

<img src="sdpics\25.png" alt="image-20240313202332685" style="zoom:33%;" />

**要点：**

<img src="sdpics\helper15.png" alt="image-20240313202441649" />

### 2.1.9 森林环境地面

#### 2.1.9.1 基础地面

项目结构

<img src="sdpics\38.png" alt="image-20240419102347034" />



岩石侵蚀新做法：

<img src="sdpics\26.png" alt="image-20240415190415165" />

pyrimad做基底的Tile Sampler，使用Non Uniform Direction Warp制作蚀刻

<img src="sdpics\27.png" alt="image-20240415190556860" />

提取shadow后量化灰度图，使用edge detect+bevel+slope blur继续雕刻边缘。

<img src="sdpics\28.png" alt="image-20240415190814414" />

和原图结合后再通过multiply叠加到石头原图上，得到类似如下的侵蚀效果。

<img src="sdpics\29.png" alt="image-20240415190928014" />

第二级的石头侵蚀：

<img src="sdpics\30.png" alt="image-20240415191033609" />

#### 2.1.9.2 散布的植被

##### 小碎石

Tile Sampler + Flood Fill + Slope Blur 制作基本型

Cell4 + Min Blend 制作大裂缝

Tile Sampler + Subtract Blend 制作表面侵蚀孔洞

<img src="sdpics\37.png" alt="image-20240419101931848" style="zoom:50%;" />

##### 小碎花

Shape + Directional Blur 制作花瓣基本型

Shape + Transform 制作叶片基本型

Spline Bridge 制作叶片脉络

Splatter Circular 进行形状组合

Alpha Merge 只提取形状的颜色，去除背景色方便主图使用

<img src="sdpics\39.png" alt="image-20240419102412359" style="zoom:50%;" />

##### 小树枝

Grunge Map005 + Directional Blur + Trapezoid Transform 制作树枝及其组合基本型

Normal + Vector Blur + High Pass 制作树枝脉络

Curvature Smooth + Curvature Sobel + Gradient Map 制作树枝颜色

<img src="sdpics\40.png" alt="image-20240419102721717" style="zoom: 50%;" />

##### 小苔藓

这个制作形态比较开放，制作好基本形状，使用Shape Splatter散布一簇一簇的苔藓群即可，仅提供参考图。

<img src="sdpics\41.png" alt="image-20240419102947685"  />

##### 蕨类植被

主图里还做了两种蕨类植被。

![image-20240419103126371](sdpics\42.png)

第一种比较好理解，以歪曲的树枝做遮罩散布叶片，然后transform调整位置blend，对树枝和和全套形状做两套shape splatter 以便于着色。

<img src="sdpics\43.png" alt="image-20240419103309071" style="zoom:67%;" />

第二种的主要思想是Directional Warp后的wave去切割叶片。再去splatter circular，其余处理手法和第一种蕨类植被一致。在着色时，叶片边缘捎带点黄色或比所选颜色更浅的颜色。

<img src="sdpics\44.png" alt="image-20240419103412729" />

#### 2.1.9.3 Final Result

<img src="sdpics\35.png" alt="image-20240419101252580" style="zoom: 38.3%;" />

<img src="sdpics\36.png" alt="image-20240419101323308" style="zoom: 50%;" />

#### 2.1.9.4 With Water-Swamp

<img src="sdpics\31.png" alt="image-20240419100550542" style="zoom:47%;" />

<img src="sdpics\34.png" alt="image-20240419100851436" style="zoom:56%;" />

<img src="sdpics\33.png" alt="image-20240419100745315" style="zoom:50%;" />

#### 2.1.9.4 With Snow

<img src="sdpics\32.png" alt="image-20240419100639584" style="zoom:50%;" />



### 2.2.1 布料

#### 2.2.1.1 印花绸棉布

<center>
    <img src="sdpics\18.png" alt="image-20240306204657927" style="zoom: 33%;" />
    <img src="sdpics\19.png" alt="image-20240306204753478" style="zoom:34%;" />

**要点：**


- 找到“针线”，通常可以使用weave。

<img src="sdpics\helper11.png" alt="image-20240306205852483" style="zoom:50%;" />

- 分清层次，一般有底料，纹案，二者使用不同的走线。通常需要使用纹案mask去warp其走线制造边缘鼓起的感觉。
- Anisotropic noise可以增加针线的横向或纵向走线感。
- 细节1：布料的褶皱，可以使用多重creased混合制作

<img src="sdpics\helper12.png" alt="image-20240306205959342" style="zoom:50%;" />

- 细节2：着色时使用noise产生随机颜色深浅，更符合布料观感

<img src="sdpics\helper13.png" alt="image-20240306210058451" style="zoom:50%;" />

- 细节3：适当增加金属度和平滑度可以增加布料质感

<img src="sdpics\helper10.png" alt="image-20240306205618716" style="zoom:50%;" />

#### 2.2.1.2 皮革旧沙发

<img src="sdpics\22.png" alt="image-20240308192930557" style="zoom:50%;" />

**要点：**

- 走针地方的褶皱和破旧感要体现
- 布料表面有磨损

#### 2.2.1.3 羊毛饰品织物

<img src="sdpics\23.png" alt="image-20240312171329083" style="zoom:50%;" />

#### 2.2.1.4 藤蔓稻穗织物

<img src="sdpics\45.png" alt="image-20240424192855358" style="zoom: 50%;" />

**要点：**

- 利用spline制作叶片的形状和茎秆的形状，两次scatter on spline得到沿branch分布的卷积茎。每次扩大spiral半径后截取不同的start和end再叠加可以得到连续的卷积形状。

<img src="sdpics\helper16.png" alt="image-20240424193500582" style="zoom: 33%;" />

### 2.2.2 镶嵌图案

#### 2.2.2.1 Mosaic地砖

<img src="sdpics\20.png" alt="image-20240307182439774" style="zoom:33%;" />

**要点：**

- 天马行空的图案
- mask to path-path to spline-scatter on spline grayscale
- 灵活运用threshold填充图案未散布的区域
- flood fill 梯度和灰度用法，调整颜色用法和以前一样
- 点睛之笔，随机方块的高光（金属度和平滑度）

#### 2.2.2.2 广场鲸鱼

<img src="sdpics\21.png" alt="image-20240307202635500" style="zoom:50%;" />

**要点：**

- SVG描绘图案



## 3.0 IDEA

### 3.1.0 少儿频道标志
<img src="sdpics\00.jpg" alt="Alt text" />
Polygon1:
![Alt text](O:\term9\notes\notes\sdpics\01.jpg)

### 3.1.1 皮革面料

<img src="sdpics\s_shapes-2.jpg" alt="image-20231218140036158" />

该方法可以表现出皮革因拉扯产生的交界处反光。

## 4.0 世界图案博物馆

### 4.0.1 丝上手绘 20世纪末 美国

图案分解：

1. 类圆角矩形和绸带线粒体
2. 短促多色笔触
3. 蓝色细胞，周围有紫色笔触
4. 双重松树叶
5. 不规则橙色线条
6. 黄色块状

颜色：暖色在上，冷色打底，底色为白黄色
