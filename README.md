# GEN
# FADN
This repository contains an implementation of Graph Enhanced Multi-Modal Network of 4-D Radar-Camera Fusion for 3D Object Detection. As shown in the figure below, the model architecture consists of three major components: ROI generator, fusion module, and fusion-based detection network.
![image](https://github.com/denyz/GEN/assets/18696187/9050f055-1674-4e7f-8955-f3f7d07eaf33)

__Note: It is built upon pytorch-geometric and provides usage with the VOD and nuScenes dataset._
<br>

# Result
Results of our model for 3D object detection on both the [VOD](https://intelligent-vehicles.org/) and the [nuScenes](https://www.nuscenes.org/) dataset. 
### 3D Object Detection in VOD
|    VOD Model      |      mAP      |    mAOS   |     Car          | Pedestrian  | Cyclist  |      
|-------------------|---------------|-----------|------------------|-------------|----------|
|      ours         |      69.3     |   59.8    |     80.7         |   53.6      |  73.5    |
|Pointpillars(Radar)|      63.0     |   56.8   |      74.1         |   47.8      |  67.1    |

![image](https://github.com/denyz/GEN/assets/18696187/04000001-319d-4b7d-b399-c3f2b00334ab)

<br>

# Prerequisites
- OS: Ubuntu 20.04 LTS
- CUDA: 11.3
- cuDNN: 8

# Preparation
Inside the project folder create a "FADN/data" folder and within this folder, create a "data/output" subfolder. The trained models and evaluations will be stored in that folder. Depending on the desired dataset, create the following additional subfolders inside the "data" folder:
```
datasets/VOD/
datasets/nuScenes/
datasets/radarscenes（later...）
```
In a second step follow the instructions of the KITTI and nuScenes websites to download and store the datasets in the created subfolders.

Finally, clone this repository into the project folder using the command:

```
git clone https://github.com/denyz/GEN.git
```

<details>
<summary>If you use the KITTI dataset, your folder structure should now look like this: </summary>

```
|  
+---GEN/  
|   |  
|   +---data/  
|   |   |  
|   |   +---datasets/  
|   |   |   |
|   |   |   +---VOD/
|   |   |   |   +---raw
|   |   |   |   |   +---radar
|   |   |   |   |   |   +---ImageSets/
|   |   |   |   |	|   |   +---train.txt
|   |   |   |   |	|   |   +---val.txt
|   |   |   |   |	|   |   +---test.txt
|   |   |   |   |	|   |   +---full.txt
|   |   |   |   |   |   +---trainning/
|   |   |   |	|   |   +---calib/
|   |   |   |   |   |   +---image_2
|   |   |   |   |   |   +---image_3
|   |   |   |   |   |   +---label_2
|   |   |   |   |   |   +---planes
|   |   |   |	|   |   +---velodyne
|   |   |   |	|   |   +---decorated_lidar
|   |   |   |   +---testing/
|   |   |   |   |   |   +---calib/
|   |   |   |   |   |   +---image_2
|   |   |   |   |   |   +---image_3
|   |   |   |   |   |   +---planes
|   |   |   |   |   |   +---velodyne
|   |   |   |
|   |   |   |   +---nuScenes
|   |   |   |   +---
|   |   |   |   +---
|   |   |
|   +---tools/  
| 
.
.
.
+---...
```
</details>
<br>

## Install
FADN is a LiDAR-camera fusion 3D detection framework. It supports many popular datasets like KITTI and nuscenes. To install the FADN please first install its requirements.txt. And as we modify some parts of the OpenPCDet LIB to support the decorated KITTI dataset. **We packaged the development environment into docker, which will be released later.** To install it, run the following commands.

```
$ python setup.py develop
```

##  Usage
The overall pipeline is divided into three major steps. 

- Creation of a decorated dataset from the raw KITTI or nuScenes dataset
- Creation and training of a model based on the created decorated dataset
- Evaluation of the trained model

The settings of all three steps are defined in a unified configuration file, which must consequently be created first.
### 1. Create a decorated dataset
the decorated dataset needs to be created by converting the lidar point clouds of the raw datasets to a decorated data structure. This will generate the decorated_lidar folder in the dataset. To do this, execute the following command inside the docker container: 
```
$ cd FADN
$ python tools/decorating.py
```
```
usage:          decorating.py [--dataset] [--config]

arguments:
    --dataset   Path to the raw (RadarScenes/nuScenes) dataset.
    --config  Parameters to the created decorated dataset.
```

Create the KITTI PKL
```
python -m pcdet.datasets.kitti.kitti_dataset create_kitti_infos tools/cfgs/dataset_configs/decorate_kitti_dataset.yaml
```

### 2. Create and train a model
Next step, you can use the created decorated dataset to train a model. To do this, run the following command: 
```
$ python -m pcdet.datasets.kitti.decorate_kitti_dataset create_kitti_infos tools/cfgs/dataset_configs/decorate_kitti_dataset.yaml
$ cd tools
```

```
usage:   train.py [--data] [--results] [--config]
$ python tools/train.py --cfg_file cfgs/kitti_models/FADN_decorated.yaml
```

### 3. Evaluate a VOD trained model 
Finally, you can evaluate a trained model using the following command in **kittiEval**:
```
usage:   ./eval_detection_3d_offline [gt_dir] [result_dir]
```
The evaluation metrics include :    
- Overlap on image (AP)
- Oriented overlap on image (AOS)
- Overlap on ground-plane (AP)
- Overlap in 3D (AP)

Within the provided "results" folder a new "data" and "plot" folder is created, in which the evaluation results are saved.
