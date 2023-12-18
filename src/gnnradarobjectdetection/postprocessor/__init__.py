from typing import Dict

from gnnradarobjectdetection.postprocessor.evaluation import Evaluator
from gnnradarobjectdetection.postprocessor.nuscenes.evaluation import NuscenesEvaluator
from gnnradarobjectdetection.postprocessor.radarscenes.evaluation import RadarscenesEvaluator
from gnnradarobjectdetection.postprocessor.VOD.evaluation import VODEvaluator

evaluation_selector: Dict[str, Evaluator] = {
    "radarscenes": RadarscenesEvaluator,
    "nuscenes": NuscenesEvaluator,
    "VOD": VODEvaluator
}
