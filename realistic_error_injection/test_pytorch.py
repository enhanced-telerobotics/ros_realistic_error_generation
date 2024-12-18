import os
from ament_index_python.packages import get_package_share_directory
import torch
from kincalib.Learning.Dataset import Normalizer
from kincalib.Learning.Models import BestMLP2
from kincalib.Learning.NoiseGenerator import NetworkNoiseGenerator
from pathlib import Path

def load_noise_generator(package_share_directory: Path, input_features: int) -> NetworkNoiseGenerator:
    weights_path = package_share_directory / "final_weights.pth"
    input_normalizer_path = package_share_directory / "input_normalizer.json"
    output_normalizer_path = package_share_directory / "output_normalizer.json"
    assert weights_path.exists(), f"Weights path {weights_path} does not exist"
    assert input_normalizer_path.exists(), f"{input_normalizer_path} does not exist"
    assert output_normalizer_path.exists(), f"{output_normalizer_path} does not exist"

    noise_generator = NetworkNoiseGenerator.create_from_files(
        weights_path, input_normalizer_path, output_normalizer_path, input_features
    )
    return noise_generator

def test_pytorch(package_share_directory:Path):
    # Construct the full path to the weights
    weights_path = package_share_directory / 'final_weights.pth'
    
    print(f"loading weights from {weights_path}")
    print(f"weights paths exist? {weights_path.exists()}")

    # Load the weights
    print("test loading model:")
    model = torch.load(weights_path, weights_only=True)

    print("Test function completed .............")
    

def simple_inference(package_share_directory: Path):
    noise_generator = load_noise_generator(package_share_directory, 12) 

    previous_jp = [[0.2,0.2,0.125,0.2,0.2,0.2]]
    sample_jp = [[0.2,0.2,0.125,0.2,0.2,0.2]]
    network_input = previous_jp + sample_jp
    cfg = dict(include_prev_measured=True)

    corrupted_jp, offset = noise_generator.corrupt_jp_batch(sample_jp, previous_jp, cfg)

    print("corrupted_jp") 
    print(corrupted_jp)
    print("offset")
    print(offset)

def main():
    package_share_directory = Path(get_package_share_directory('realistic_error_injection'))
    package_share_directory = package_share_directory / 'resources/sample_neural_net_dataset4' 

    test_pytorch(package_share_directory)
    simple_inference(package_share_directory)
