from gnnradarobjectdetection.preprocessor.radarscenes.dataset_creation import RadarScenesGraphDataset
from gnnradarobjectdetection.preprocessor.nuscenes.dataset_creation import NuScenesGraphDataset
from gnnradarobjectdetection.preprocessor.VOD.dataset_creation import VODGraphDataset

from gnnradarobjectdetection.preprocessor.radarscenes.configs import RadarScenesDatasetConfiguration
from gnnradarobjectdetection.preprocessor.nuscenes.configs import NuScenesDatasetConfiguration
from gnnradarobjectdetection.preprocessor.VOD.configs import VODDatasetConfiguration

dataset_selector = {
    'radarscenes': RadarScenesGraphDataset,
    'nuscenes': NuScenesGraphDataset,
    'VOD': VODGraphDataset
}

config_selector = {
    'radarscenes': RadarScenesDatasetConfiguration,
    'nuscenes': NuScenesDatasetConfiguration,
    'VOD': VODDatasetConfiguration
}
