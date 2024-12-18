from ament_index_python.packages import get_package_share_directory
import torch
from kincalib.Learning.NoiseGenerator import NetworkNoiseGenerator
from pathlib import Path
from realistic_error_injection.utils import (
    load_noise_generator,
    get_path_to_torch_model,
)


def test_pytorch(package_share_directory: Path):
    # Construct the full path to the weights
    weights_path = package_share_directory / "final_weights.pth"

    print(f"loading weights from {weights_path}")
    print(f"weights paths exist? {weights_path.exists()}")

    # Load the weights
    print("test loading model:")
    model = torch.load(weights_path, weights_only=True)

    print("Test function completed .............")


def simple_inference(package_share_directory: Path):
    noise_generator, cfg = load_noise_generator(package_share_directory)

    previous_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]
    sample_jp = [[0.2, 0.2, 0.125, 0.2, 0.2, 0.2]]

    corrupted_jp, offset = noise_generator.corrupt_jp_batch(sample_jp, previous_jp, cfg)

    print("input")
    print(sample_jp)
    print("corrupted_jp")
    print(corrupted_jp)
    print("offset")
    print(offset)


def main():
    torch_model_path = get_path_to_torch_model()

    test_pytorch(torch_model_path)
    simple_inference(torch_model_path)
