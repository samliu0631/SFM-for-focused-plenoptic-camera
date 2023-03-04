# SFM-for-focused-plenoptic-camera
Structure from motion algorithm suitable for focused plenoptic camera.
# Installation
Run \SFM\3rdPart\toolbox\vl_setup.m to install the SIFT detector.

Run \SFM\PlenopticTool\AddToolboxPath.m  to install the toolbox.

# Data
Google Driver

https://drive.google.com/drive/folders/13bsnJd4Lz8fxNklz5cZvlJ8sVX2eyuDS?usp=sharing

Download the dataset and put it anywhere you want.
# Demo 
Change the dataset path in \SFM\Demo\SFMTankCircle.m(Line 3).

`MainPath   = 'The path where you put the dataset' ;  `


Run \SFM\Demo\SFMTankCircle.m.

# Paper
```
If you find our method usedful, please cite our paper.
@article{刘青松2021聚焦型光场相机等效多相机模型及其运动恢复结构应用,
  title={聚焦型光场相机等效多相机模型及其运动恢复结构应用},
  author={刘青松 and 谢晓方 and 张烜喆 and 田瑜 and 许晓军},
  journal={光学学报},
  year={2021}
}
```
https://www.researching.cn/ArticlePdf/m00006/2021/41/3/0315001.pdf
