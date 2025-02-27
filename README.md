# DRIVE_AGAIN

## Run project

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

## UI setup

TODO
