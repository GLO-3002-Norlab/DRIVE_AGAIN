import pytest

from DRIVE_AGAIN.sampling import *


@pytest.fixture
def random_sampling() -> RandomSampling:
    """Set up a random sampling for testing"""
    return RandomSampling()


def test_sample_command(random_sampling: RandomSampling):
    result = random_sampling.sample_command()

    assert result is not None
