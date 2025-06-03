from pathlib import Path
from typing import Dict, Tuple
from kincalib.Learning.NoiseGenerator import NetworkNoiseGenerator
import json


def load_noise_generator(
    package_share_directory: Path,
) -> Tuple[NetworkNoiseGenerator, Dict]:

    weights_path = package_share_directory / "final_weights.pth"
    input_normalizer_path = package_share_directory / "input_normalizer.json"
    output_normalizer_path = package_share_directory / "output_normalizer.json"
    dataset_info = json.load(open(package_share_directory / "dataset_info.json", "r"))

    assert weights_path.exists(), f"Weights path {weights_path} does not exist"
    assert input_normalizer_path.exists(), f"{input_normalizer_path} does not exist"
    assert output_normalizer_path.exists(), f"{output_normalizer_path} does not exist"

    if dataset_info["include_prev_measured"]:
        dataset_info["input_features"] = 12
    else:
        dataset_info["input_features"] = 6

    noise_generator = NetworkNoiseGenerator.create_from_files(
        weights_path,
        input_normalizer_path,
        output_normalizer_path,
        dataset_info["input_features"],
    )

    return noise_generator, dataset_info


def get_path_to_torch_model() -> Path:
    package_share_directory = Path(__file__).parent

    package_share_directory = (
        package_share_directory / "resources/sample_neural_net_dataset4"
    )

    return package_share_directory
