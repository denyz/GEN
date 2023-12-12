# GEN
This repository contains an implementation of Graph Enhanced Multi-Modal Network of Radar-Camera Fusion for 3D Object Detection. As shown in the figure below, the model architecture consists of three major components: point cloud augmentation module, fusion module, and fusion-based detection network.
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
Inside the project folder create a "GEM/data" folder and within this folder, create a "data/result" subfolder. The trained models and evaluations will be stored in that folder. Depending on the desired dataset, create the following additional subfolders inside the "data" folder:
```
datasets/VOD/
datasets/nuScenes/
datasets/process (for graph construction)
datasets/radarscenes（later...）
```
In a second step follow the instructions of the VOD and nuScenes websites to download and store the datasets in the created subfolders.

Finally, clone this repository into the project folder using the command:

```
git clone https://github.com/denyz/GEN.git
```

<details>
<summary>If you use the VOD dataset, your folder structure should now look like this: </summary>

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
|   |   |   |	|   |   |   +---calib/
|   |   |   |   |   |   |   +---image_2
|   |   |   |   |   |   |   +---label_2
|   |   |   |	|   |   |   +---pose
|   |   |   |	|   |   |   +---velodyne
|   |   |   |   |   |   +---testing/
|   |   |   |   |   |   |   +---calib/
|   |   |   |  |    |   |   +---image_2
|   |   |   | |     |   |   +---pose
|   |   |   | |     |   |   +---velodyne
|   |   |   |   |   +---lidar
|   |   |   |   |   +---radar_3frames
|   |   |   |   |   +---radar_5frames
|   |   |   |   |   +---label_2
|   |   |   |
|   |   |   |   +---nuScenes
|   |   |   |   +---...
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
GEN is a Radar-camera fusion 3D detection framework. It supports many popular datasets like VOD and nuscenes. To install the GEN, please first go ahead and install its requirements.txt. And as we modify some parts of the OpenPCDet LIB to support the decorated VOD dataset. **We packaged the development environment into docker, which will be released later.** To install it, run the following commands.

```
$ pip install requirements.txt
```

##  Usage
The overall pipeline is divided into three major steps. 

- Creation of a graph dataset from the raw VOD or nuScenes dataset
- Creation and training of a model based on the graph dataset
- Evaluation of the trained model

The settings of all three steps are defined in a unified configuration file, which must consequently be created first.
### 1. Create a graph dataset
the graph dataset needs to be created by converting the radar point clouds of the raw datasets to a graph data structure. This will generate the decorated_lidar folder in the dataset. To do this, execute the following command: 
```
$ cd GEN
$ python3 src/gnnradarobjectdetection/create_dataset.py --dataset ${path_to_raw_dataset_folder}$ --config ${path_to_config_file}$
```
```
usage:          create_dataset.py [--dataset] [--config]

arguments:
    --dataset   Path to the raw (VOD/nuScenes) dataset
    --config    Path to the created configuration.yml file
```

### 2. Create and train a model
Next step, you can use the created graph dataset to train a model. To do this, run the following command: 
```
$ python3 src/gnnradarobjectdetection/train.py --data ${path_to_graph_dataset_folder}$ --results ${path_to_results_folder}$ --config ${path_to_config_file}$

$ cd tools
```
```
usage:             train.py [--data] [--results] [--config]

arguments:
    --data         Path to the created graph-dataset
    --results      Path to the created "results" folder
    --config       Path to the created configuration.yml file
```

### 3. Evaluate a trained model 
Finally, you can evaluate a trained model using the following command:
```
$ python3 evaluate.py --data ${path_to_graph_dataset_folder}$ --model ${path_to_model_folder}$ --config ${path_to_config_file}$

arguments:
    --data         Path to the created graph dataset (The same as used for the training of the model to evaluate)
    --model        Path to the folder in which the trained model is saved
    --config       Path to the created configuration.yml file
```
The evaluation metrics included in VOD:    
- Average Precision (AP)
- Average Orientation Similarity (AOS)

Within the provided "results" folder a new "data" and "plot" folder is created, in which the evaluation results are saved.
