# DRIVE - Data-driven Robot Input Vector Exploration

[![DOI](https://zenodo.org/badge/DOI/10.48550/arxiv.2309.110718.svg)](https://doi.org/10.48550/arXiv.2309.10718)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue?labelColor=blue&logo=ROS)](https://docs.ros.org/en/humble)

DRIVE is an open-source uncrewed ground vehicle (UGV) training dataset gathering protocol.
This protocol automates the task of driving the UGV to gather a training dataset, then used to train a motion model. The resulting model can then be used for controllers.

[![SNOW WILN deployment](https://img.youtube.com/vi/tBCtC7WolL4/0.jpg)](https://www.youtube.com/watch?v=tBCtC7WolL4)

👉 [See on Youtube](https://www.youtube.com/watch?v=tBCtC7WolL4)

## Documentation and Tutorials

Quick link for the [documentation](https://drive-again.readthedocs.io/en/latest/).

Those tutorials are written using Markdown syntax and stored in the project's `/docs` folder. Their scope ranges from how to use the project to technical documentation for more experienced developer on how to extend the codebase.

## Installation

Although we suggest to use the tutorials, here is a quick version of it

### Natively

1. Create venv with python 3.10
2. Install dependencies

```sh
pip install -e .
```

3. Run the project

```sh
python src/DRIVE_AGAIN/app.py
```

### Docker

1. Build the dockerfile

```sh
docker build -t drive_again .
```

2. Run the app

```sh
docker run -p 5000:5000 drive_again
```

## Test

```sh
python -m pytest
```

## Coverage
```sh
coverage run --source=DRIVE_AGAIN -m pytest
```
and then run
```sh
coverage report
```
or alternatively, for a nicer result, run
```sh
coverage html
```
and open the generated HTML file

> Note that in order for changes in the base code to be applied, you will need to run `pip install .` between test runs.

## Citing

If you use DRIVE in an academic context, please cite [our preprint](https://www.researchgate.net/publication/374023495_DRIVE_Data-driven_Robot_Input_Vector_Exploration):

```bibtex
@misc{baril2023drive,
      title={DRIVE: Data-driven Robot Input Vector Exploration},
      author={Dominic Baril and Simon-Pierre Deschênes and Luc Coupal and Cyril Goffin and Julien Lépine and Philippe Giguère and François Pomerleau},
      year={2023},
      eprint={2309.10718},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Datasets

The datasets used in our paper are publicly available.
[Follow this link to download them in Pandas Dataframe format](https://github.com/norlab-ulaval/Norlab_wiki/wiki/DRIVE-datasets).
